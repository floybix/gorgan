(ns gorgan.core
  (:use [cljbox2d core joints testbed]
        [cljbox2d.vec2d :only [TWOPI PI in-pi-pi
                               angle* angle-left?]])
  (:require [quil.core :as quil]))

;; a creature
(def ^:dynamic *it*)

;; bound as context within function (decide!)
(def ^:dynamic *decision-path*)

;; TODO these should be nested in *it* :

;; energy reserves in joules
(def energy (atom nil))

(def last-decisions (atom []))
;; a vector corresponding to last-decisions
(def entered-decisions (atom []))

;; @pos-history has a staggered series of queues.
;; we keep one second (/ 1 *timestep*) values in :steps
;; every 1 second the oldest :steps is added to :secs
;; we keep 60 seconds in :secs ... etc
(def pos-history (atom nil))

(defn reset-pos-history!
  [pos]
  (reset! pos-history
          (let [q (clojure.lang.PersistentQueue/EMPTY)]
            {:steps (into q (repeat (/ 1 *timestep*) pos))
             :secs (into q (repeat 60 pos))
             :mins (conj q pos)
             :last-sec @world-time
             :last-min @world-time})))

(defn pos-at-time-ago
  [tspan]
  (let [{:keys [steps secs mins]} @pos-history]
    (cond
     (< tspan 1) (nth steps (int (/ tspan *timestep*)))
     (< tspan 60) (nth secs (dec (int tspan)))
     :else (nth mins (dec (min (count mins) (int (/ tspan 60))))))))

(defn pos-history-step!
  [pos]
  (swap! pos-history
         (fn [{:keys [steps secs mins last-sec last-min], :as m}]
           (let [next-min? (> @world-time (+ last-sec 60))
                 next-sec? (> @world-time (+ last-sec 1))]
             (merge m
                    {:steps (conj (pop steps) pos)}
                    (when next-min?
                      {:mins (conj mins (peek secs))
                       :last-min @world-time})
                    (when next-sec?
                      {:secs (conj (pop secs) (peek steps))
                       :last-sec @world-time}))))))

(def foodlist (atom []))

(def ^:dynamic *debug* true)

;; ## DSL for movement scheme

;; ### grammar of expressions safe for use in tests (no side effects!)

;; not used currently

(def pure-fn-types
  {'#(@energy) [:num []]
   'x-velocity [:pos-num [:pos-num]]
   'y-velocity [:pos-num [:pos-num]]
   'head-angle [:num []]
   'time-in-state [:pos-num []]
   'food-left [:int [:pos-num]]
   'food-right [:int [:pos-num]]
   'food-within [:int [:pos-num]]
   'j [:joint [:side :int]] ; only 0 or 1 for now
   'on-ground? [:logical [:joint]]  ;; TODO try this!
   'j-speed [:num [:joint]]
   'j-angle [:num [:joint]]
   'leg-offset-x [:num [:joint]]
   '< [:logical [:num :num]]
   '> [:logical [:num :num]]
   'pos? [:logical [:num]]
   'neg? [:logical [:num]]
   'zero? [:logical [:num]]
   'not [:logical [:logical]]
   'and [:logical [:logical :logical]]
   'or [:logical [:logical :logical]]
   'min [:num [:num :num]]
   'max [:num [:num :num]]
   'abs [:pos-num [:num :num]]
   'min-abs-limit [:num [:num :pos-num]]
   'max-abs-limit [:num [:num :pos-num]]
   '- [:num [:num :num]]
   '+ [:num [:num :num]]
   '* [:num [:num :num]]
   '/ [:num [:num :num]]
   'if [:num [:logical :num :num]] ;; ??? rely on :test?
   })

(def effect-fn-types
  {'j-off! [:joint]
   'j-on! [:joint]
   'j-speed! [:joint :num]
   'j-angle! [:joint :num]
   'j-world-angle! [:joint :num]
   'j-maxtorque! [:joint :pos-num]
   })

(defn abs
  [x]
  (if (pos? x) x (- x)))

(defn min-abs-limit
  [x abs-min]
  (* (min (abs x) abs-min) (if (pos? x) 1 -1)))

(defn max-abs-limit
  [x abs-max]
  (* (max (abs x) abs-max) (if (pos? x) 1 -1)))

;; TODO: should these divide by tspan?

(defn x-velocity
  "Horizontal velocity of head integrated over some time span."
 [tspan]
 (- (first (position (:head *it*)))
    (first (pos-at-time-ago tspan))))

(defn y-velocity
  "Vertical velocity of head integrated over some time span."
 [tspan]
 (- (second (position (:head *it*)))
    (second (pos-at-time-ago tspan))))

(defn head-angle
  []
  (angle (:head *it*)))

(defn time-in-state
 "Time since last deviation from the current node in the movement
  scheme tree. Depends on (and detects) the node from which it was
  called. Note that the final decision node may change without
  deviation from an intermediate decision node. Consequently the
  top-level node can never be deviated from, and its time-in-state is
  never reset."
 []
 (if (empty? *decision-path*)
   @world-time
   (let [depth (count *decision-path*)]
     (if (= *decision-path* (take depth @last-decisions))
       (- @world-time (nth @entered-decisions (dec depth)))
       0.0))))

(defn food-within
  "Counts food particles within given distance left or right.
   Vertical range is the same as ped height."
  ([dist]
     (food-within dist dist))
  ([ldist rdist]
     (let [[x y] (position (:head *it*))
           ;; returns a vector of fixtures:
           fxx (query-aabb (aabb [(- x ldist) y]
                                 [(+ x rdist) 0]))]
       (count (filter #(:is-food? (user-data %)) fxx)))))

(defn food-left
  "Counts food particles within given distance to left.
   Vertical range is the same as ped height."
  [dist]
  (food-within dist -0.1))

(defn food-right
  "Counts food particles within given distance to right.
   Vertical range is the same as ped height."
  [dist]
  (food-within -0.1 dist))

(defn j
  "selects a joint or multiple joints (a sequence)"
  ([]
     (concat (j :left) (j :right)))
  ([side]
     (:joints (get *it* side)))
  ([side index]
     (nth (j side) index)))


;; queries on collections of joints: should we
;;   a) not allow
;;   b) return a sequence of speeds
;;   c) return average of speeds?

(defn on-ground?
  "tests whether a leg (extending out from given joint) is contacting
   the ground."
  [jt]
  (let [cset (contacting (body-b jt))]
    (cset @ground-body)))

(defn j-speed
  "target motor speed for joint."
  [jt]
  (motor-speed jt))

(defn j-angle
  [jt]
  (joint-angle jt))

(defn j-world-angle
  [jt]
  (angle (body-b jt)))

(defn leg-offset-x
  [jt]
  (- (first (position (body-b jt)))
     (first (position (:head *it*)))))

(defn j-off!
  [jt]
  (if (coll? jt)
    (dorun (map j-off! jt))
    (enable-motor! jt false)))

(defn j-on!
  [jt]
  (if (coll? jt)
    (dorun (map j-on! jt))
    (enable-motor! jt true)))

(defn j-speed!
  [jt speed]
  (if (coll? jt)
    (dorun (map #(j-speed! % speed) jt))
    (do
      (enable-motor! jt true)
      (motor-speed! jt speed))))

(defn j-angle!
  "set speed according to difference from target angle relative to
   attached body (??)" ;; TODO think about this
  [jt ang]
  (if (coll? jt)
    (dorun (map #(j-angle! % ang) jt))
    (do
      (enable-motor! jt true)
      (let [ang-diff (in-pi-pi (- ang (joint-angle jt)))]
        (motor-speed! jt (* 3 ang-diff))))))

(defn j-world-angle!
  "set speed according to difference from target world angle (of
   attached outer body)"
  [jt ang speed-factor]
  (if (coll? jt)
    (dorun (map #(j-world-angle! % ang) jt))
    (do
      (enable-motor! jt true)
      (let [ang-diff (in-pi-pi (- ang (angle (body-b jt))))]
        (motor-speed! jt (* speed-factor ang-diff))))))

(defn j-maxtorque!
  [jt torque]
  (if (coll? jt)
    (dorun (map #(j-maxtorque! % torque) jt))
    (max-motor-torque! jt torque)))

(defn decide!
  ([scheme]
     (decide! scheme []))
  ([scheme path]
     (binding [*ns* (find-ns 'gorgan.core)
               *decision-path* path]
       (when-let [actions (:do scheme)]
         (eval actions))
       (if-let [test-form (:test scheme)]
         (let [result (boolean (eval test-form))
               next-scheme (if result
                             (:when-true scheme)
                             (:when-false scheme))]
           (if next-scheme
             (decide! next-scheme (conj path result))
             path))
         path))))

(defn path-of-tests
  [scheme path]
  (loop [s scheme
         p path
         tests []]
    (if s
      (let [res (first path)
            next-s (if res (:when-true s) (:when-false s))]
        (recur next-s (next path) (conj tests (:test s))))
      tests)))

(def first-basic-scheme
  {
   :test '(> (food-within 1) 0)
   :when-true
   {
    :do ['(j-off! (j))]
    :test '(> (time-in-state) 3)
    :when-true
    {
     :do ['(j-speed! (j :left) 3)]
     }
    }
   :when-false
   {
    :do ['(j-world-angle! (j :left 1) 0 5)
         '(j-world-angle! (j :right 1) 0 5)]
    :test '(> (food-left 10) 0)
    :when-true
    {
     :do ['(j-speed! (j :left 0) 2)
          '(j-speed! (j :right 0) 2)]
     :test '(> (time-in-state) 5)
     :when-true
     {
      :do ['(j-speed! (j :left 1) 2)
           '(j-speed! (j :right 1) 2)]
      }
     }
    :when-false
    {
     :test '(> (food-right 10) 0)
     :when-true
     {
      :do ['(j-speed! (j :left 0) -2)
           '(j-speed! (j :right 0) -2)]
      :test '(> (time-in-state) 5)
      :when-true
      {
       :do ['(j-speed! (j :left 1) -5)
            '(j-speed! (j :right 1) -5)]
       }
      }
     :when-false
     {
      :do ['(j-world-angle! (j :left 0) 0 2)
           '(j-world-angle! (j :right 0) 0 2)]
      :test '(> (time-in-state) 2)
      :when-true
      {
       :do ['(j-speed! (j :left 0) (min-abs-limit (head-angle) 0.2))
            '(j-speed! (j :right 0) (min-abs-limit (head-angle) 0.2))]
       }
      }
     }
    }
   }
  )

(defn make-food
  "Creates and returns a food particle.
   :user-data of the fixture is a map `{:is-food? true}`
   :user-data of the body is also a map `{:is-food? true}"
  [position joules]
  (body! {:position position
          :user-data {:is-food? true, :joules joules}}
         {:shape (polygon [[0 0.6] [-0.25 0] [0.25 0]])
          :density 5 :restitution 0 :friction 0.8
          :user-data {:is-food? true}}))

(defn make-leg
  [body dir group-index]
  (let [angle (angle* dir)
        at (edge-point body angle)
        thigh (body! {:position at}
                     {:shape (rod [0 0] angle 3 0.1)
                      :group-index group-index})
        at-knee (edge-point thigh angle)
        calf-angle (+ angle (* 0.75 PI
                               (if (angle-left? angle) 1 -1)))
        calf (body! {:position at-knee}
                    {:shape (rod [0 0] calf-angle 5 0.1)
                     :group-index group-index})
        jt-opts {:enable-motor true
                 :motor-speed 0
                 :max-motor-torque 500}
        j0 (revolute-joint! body thigh at
                            jt-opts)
        j1 (revolute-joint! thigh calf at-knee
                            jt-opts)]
    {:bodies [thigh calf]
     :joints [j0 j1]}))

(defn make-2ped
  [position group-index]
  (let [head (body! {:position position}
                    {:shape (circle 1)
                     :group-index group-index})
        rleg (make-leg head :top-right group-index)
        lleg (make-leg head :top-left group-index)]
    {:right rleg :left lleg :head head}))

(defn poly-edges
  [vertices attrs]
  (for [[v0 v1] (partition 2 1 vertices)]
    (merge {:shape (edge v0 v1)} attrs)))

(defn setup-world! []
  (create-world!)
  (let [ground (apply body! {:type :static}
                      (poly-edges [[-100 35]
                                   [-90 5]
                                   [-70 5]
                                   [-65 10]
                                   [-62 12]
                                   [-60 10]
                                   [-40 0]
                                   [10 0]
                                   [10.5 1]
                                   [10.8 0]
                                   [30 0]
                                   [35 5]
                                   [50 5]
                                   [60 0]
                                   [62 0]
                                   [63 1]
                                   [65 25]]
                                  {:friction 1}))
        allfood (doall (for [x (range -90 60 8)]
                         (make-food [x 12] 50)))
        ped (make-2ped [0 3] -2)]
    (alter-var-root (var *it*) (fn [_] ped))
    (reset! energy 500)
    (reset-pos-history! (position (:head *it*)))
    (reset! foodlist allfood)
    (reset! ground-body ground)))

(defn draw-info-text []
  (let [{{[l0 l1] :joints} :left
         {[r0 r1] :joints} :right} *it*
        fmt (fn [x] (format "%.2f" x))
        fmt-ang (fn [x] (str (fmt (/ x PI)) "pi"))
        j-info (fn [j nm]
                 (str nm
                      ": angle = " (fmt-ang (in-pi-pi (joint-angle j)))
                      ", motor " (if (motor-enabled? j) "on" "off")
                      ", mspeed = " (fmt-ang (motor-speed j))
                      ", mtorque = " (fmt (motor-torque j))))]
    (quil/fill 255)
    (quil/text-align :left)
    (quil/text (str (j-info l0 "l0") "\n"
                    (j-info l1 "l1") "\n"
                    (j-info r0 "r0") "\n"
                    (j-info r1 "r1"))
               10 10))
  ;; draw energy reserves as bar
  (quil/rect-mode :corner)
  (quil/fill (quil/color 128 128 128))
  (quil/rect (- (quil/width) 10) 0 10 (quil/height))
  (quil/fill (quil/color 128 255 128))
  (quil/rect (- (quil/width) 10) 0 10 (* (quil/height)
                                         (/ @energy 1000))))

(defn my-key-press []
  (let [{{[l0 l1] :joints} :left
         {[r0 r1] :joints} :right} *it*
        delta (/ PI 4)]
    (case (quil/raw-key)
      \q (motor-speed! l1 (+ (motor-speed l1) delta))
      \w (motor-speed! l0 (+ (motor-speed l0) delta))
      \e (motor-speed! r0 (+ (motor-speed r0) delta))
      \r (motor-speed! r1 (+ (motor-speed r1) delta))
      
      \a (enable-motor! l1 (not (motor-enabled? l1)))
      \s (enable-motor! l0 (not (motor-enabled? l0)))
      \d (enable-motor! r0 (not (motor-enabled? r0)))
      \f (enable-motor! r1 (not (motor-enabled? r1)))

      \z (motor-speed! l1 (- (motor-speed l1) delta))
      \x (motor-speed! l0 (- (motor-speed l0) delta))
      \c (motor-speed! r0 (- (motor-speed r0) delta))
      \v (motor-speed! r1 (- (motor-speed r1) delta))

      ;; otherwise pass on to testbed
      (key-press))))

(defn my-step []
  (pos-history-step! (position (:head *it*)))
  ;; Work(joules) = torque * rotation-angle
  ;; note joules/second = watts
  (let [each-power (map power-watts (j))
        joules (* (reduce + each-power) *timestep*)]
    (swap! energy - joules))
  (let [path (decide! first-basic-scheme)]
    (when-not (= path @last-decisions)
      (when *debug*
        (println (format "@%.1f" @world-time)
                 "new decision: " path)
        (let [tests (path-of-tests first-basic-scheme path)
              nt (count tests)]
          (doseq [[i tst res] (map vector (range nt) tests path)]
            (println (apply str (repeat (* i 2) \ ))
                     (if res "T" "F") ":"
                     tst))))
      (let [n-same (count (take-while true? (map = path @last-decisions)))
            n-more (- (count path) n-same)]
        (reset! last-decisions path)
        (swap! entered-decisions
               (fn [this] (concat (take n-same this)
                                  (repeat n-more @world-time)))))))
  (doseq [other (contacting (:head *it*))
          :let [info (user-data other)]]
    (when (:is-food? info)
      (swap! energy + (:joules info))
      (destroy! other))))

(defn setup []
  (quil/frame-rate (/ 1 *timestep*))
  (setup-world!)
  (reset! step-fn my-step)
  (reset! draw-more-fn draw-info-text))

(defn -main
  "Run the sketch."
  [& args]
  (quil/defsketch the-sketch
    :title "Gorgan"
    :setup setup
    :draw draw
    :key-typed my-key-press
    :mouse-pressed mouse-pressed
    :mouse-released mouse-released
    :mouse-dragged mouse-dragged
    :size [1200 800]))
