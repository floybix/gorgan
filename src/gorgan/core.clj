(ns gorgan.core
  (:use [cljbox2d core joints testbed]
        [cljbox2d.vec2d :only [TWOPI PI in-pi-pi
                               angle* angle-left?]])
  (:require [quil.core :as quil]))

(def ^:dynamic *it* nil)

(def current-decision-path (atom nil))
(def entered-decision (atom 0))


;; ## DSL for movement scheme

;; ### grammar of expressions safe for use in tests (no side effects!)

;; not used currently

(def pure-fn-types
  {'x-velocity [:pos-num [:pos-num]]
   'head-angle [:num []]
   'time-in-state [:pos-num []]
   'food-left? [:logical [:pos-num]]
   'food-within? [:logical [:pos-num]]
   'j-speed [:num [:joint]]
   'j-angle [:num [:joint]]
   'j [:joint [:side :int]] ; only 0 or 1 for now
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
   '- [:num [:num :num]]
   '+ [:num [:num :num]]
   '* [:num [:num :num]]
   '/ [:num [:num :num]]
   })

(def effect-fn-types
  {'j-off! [:joint]
   'j-on! [:joint]
   'j-speed! [:joint :num]
   'j-angle! [:joint :num]
   'j-maxtorque! [:joint :pos-num]
   })

(defn x-velocity
 "horizontal velocity of head integrated over some time span"
 [tspan]
 )

(defn head-angle
 []
 )

(defn time-in-state
 "time since we entered the currently selected node in the movement
  scheme tree"
 []
 (- @world-time @entered-decision))

(defn food-within?
  "queries for food within given distance left or right.
   vertical range is the same as ped height."
  ([dist]
     (food-within? dist dist))
  ([ldist rdist]
     ;; returns a vector of fixtures:
     ;; TODO: test for food
     (let [[x y] (position (:head *it*))
           fxx (query-aabb (aabb [(- x ldist) y]
                                 [(+ x rdist) 0]))]
       (some #(:is-food? (user-data %)) fxx))))

(defn food-left?
  "queries for food within given distance to left.
   vertical range is the same as ped height."
  [dist]
  (food-within? dist -0.1))

(defn food-right?
  "queries for food within given distance to right.
   vertical range is the same as ped height."
  [dist]
  (food-within? -0.1 dist))

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
  [jt angle]
  (if (coll? jt)
    (dorun (map #(j-angle! % angle) jt))
    (do
      (enable-motor! jt true)
      (let [ang-diff (in-pi-pi (- angle (joint-angle jt)))]
        (motor-speed! jt (* 3 ang-diff))))))

(defn j-world-angle!
  "set speed according to difference from target world angle (of
   attached outer body)"
  [jt angle]
  (if (coll? jt)
    (dorun (map #(j-world-angle! % angle) jt))
    (do
      (enable-motor! jt true)
      (let [ang-diff (in-pi-pi (- angle (angle (body-b jt))))]
        (motor-speed! jt (* 3 ang-diff))))))

(defn j-maxtorque!
  [jt torque]
  (if (coll? jt)
    (dorun (map #(j-maxtorque! % torque) jt))
    (max-motor-torque! jt torque)))

(defn decide!
  ([scheme]
     (decide! scheme []))
  ([scheme path-taken]
     (binding [*ns* (find-ns 'gorgan.core)]
       (when-let [actions (:do scheme)]
         (eval actions))
       (if-let [test-form (:test scheme)]
         (let [result (eval test-form)
               next-scheme (if result
                             (:when-true scheme)
                             (:otherwise scheme))]
           (if next-scheme
             (decide! next-scheme (conj path-taken result))
             path-taken))
         path-taken))))

(def first-basic-scheme
  {
   :test '(food-within? 1)
   :when-true
   {
    :do ['(j-off! (j))]
    :test '(> (time-in-state) 3)
    :when-true
    {
     :do ['(j-speed! (j :left 1) 2)]
     }
    }
   :otherwise
   {
    :do ['(j-on! (j))]
    :test '(food-left? 10)
    :when-true
    {
     :do ['(j-speed! (j :left 1) 1)
          '(j-speed! (j :right 1) 1)]
     }
    :otherwise
    {
     :test '(food-right? 10)
     :when-true
     {
      :do ['(j-speed! (j :left 1) -1)
           '(j-speed! (j :right 1) -1)]
      }
     :otherwise
     {
      :do ['(j-speed! (j :left 1) 0)
           '(j-speed! (j :right 1) 0)]
      }
     }
    }
   }
  )

(defn make-food
  "Creates and returns a food particle.
   :user-data of the fixture is a map `{:is-food? true}`"
  [position]
  (body! {:position position}
         {:shape (polygon [[0 0.6] [-0.25 0] [0.25 0]])
          :restitution 0 :friction 0.8
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
                 :max-motor-torque 1000}
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

(defn setup-world! []
  (create-world!)
  (let [ground (body! {:type :static}
                      {:shape (edge [-40 0] [40 0])}
                      {:shape (edge [-40 0] [-50 20])}
                      {:shape (edge [40 0] [45 5])}
                      {:shape (edge [45 5] [60 5])}
                      {:shape (edge [60 5] [65 25])})
        allfood (doall (for [x (range -40 40 5)]
                         (make-food [x 0.6])))
        ped (make-2ped [0 3] -2)]
    (alter-var-root (var *it*) (fn [_] ped))
    (reset! ground-body ground)))

(defn draw-info-text []
  (let [{{[l0 l1] :joints} :left
         {[r0 r1] :joints} :right} *it*
        fmt (fn [x] (format "%.2f" x))
        fmt-ang (fn [x] (str (fmt (/ x PI)) "pi"))
        j-info (fn [j nm]
                 (str nm
                      ": angle = " (fmt-ang (joint-angle j))
                      ", motor " (if (motor-enabled? j) "on" "off")
                      ", mspeed = " (fmt-ang (motor-speed j))
                      ", mtorque = " (fmt (motor-torque j))))]
    (quil/fill 255)
    (quil/text-align :left)
    (quil/text (str (j-info l0 "l0") "\n"
                    (j-info l1 "l1") "\n"
                    (j-info r0 "r0") "\n"
                    (j-info r1 "r1"))
               10 10)))

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
  (let [path (decide! first-basic-scheme)]
    (when-not (= path @current-decision-path)
      (println "new decision: " path)
      (reset! current-decision-path path)
      (reset! entered-decision @world-time))
    ))

(defn setup []
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
    :size [600 500]))
