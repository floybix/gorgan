(ns gorgan.dsl
  (:use [cljbox2d core joints]
        [cljbox2d.testbed :only [ground-body]]
        [cljbox2d.vec2d :only [in-pi-pi
                               v-mag v-interp]]
        [gorgan.core :only [*it* energy
                            energy-in-eggs
                            pos-history
                            energy-history
                            energy-eaten-history
                            proposals
                            operator-chosen-at]])
  (:require [gorgan.history-queues :as hq]))

;; proposals-scheme specific stuff

(defn propose
  [op-name priority]
  (swap! proposals #(merge-with max % %2)
         {op-name priority}))

(defmacro when-all
  [conds & body]
  `(when (every? boolean ~conds)
     ~@body))

;; tree-scheme specific stuff

;; bound as context within function 'dsl/decide!
(def ^:dynamic *decision-path*)

(def last-decisions (atom []))
;; a vector corresponding to last-decisions
(def entered-decisions (atom []))

;; ## DSL for movement scheme

;; ### grammar of expressions safe for use in tests (no side effects!)

;; not used currently

(def pure-fn-types
  {'#(@energy) [:num []]
   'x-movement-offset [:pos-num [:pos-num]]
   'y-movement-offset [:pos-num [:pos-num]]
   'height [:pos-num []]
   'head-angle [:num []]
   'time-in-state [:pos-num []]
   'food-left [:int [:pos-num]]
   'food-right [:int [:pos-num]]
   'food-within [:int [:pos-num]]
   'j [:joint [:side :int]] ; only 0 or 1 for now
   'on-ground? [:logical [:joint]]  ;; TODO try this!
   'velocity [:vec []]
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
   'v-mag [:pos-num [:vec]]
   'v-interp [:vec [:vec :vec :pos-num]]
   ;; etc
   })

(def effect-fn-types
  {'j-off! [:joint]
   'j-on! [:joint]
   'j-speed! [:joint :num]
   'j-angle! [:joint :num]
   'j-world-angle! [:joint :num]
   'j-maxtorque! [:joint :pos-num]
   'lay-egg! [:pos-num]
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

(defn energy-balance
  [tspan]
  )

(defn energy-consumed
  [tspan]
  )

(defn x-movement-offset
  "Horizontal movement of head integrated over some time span."
 [tspan]
 (- (first (position (:head *it*)))
    (first (hq/val-at-time-ago pos-history tspan))))

(defn y-movement-offset
  "Vertical movement of head integrated over some time span."
 [tspan]
 (- (second (position (:head *it*)))
    (second (hq/val-at-time-ago pos-history tspan))))

(defn height
  "Vertical distance of head from the ground."
  []
  ;; TODO - need raycast?
  )

(defn velocity
  []
  (linear-velocity (:head *it*)))

(defn head-angle
  []
  (angle (:head *it*)))

(defn time-in-state
  []
  (- @world-time @operator-chosen-at))

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

(defn j-level
  "selects joints at a given level out from head, starting at 0 for
   head-attached joints."
  [index]
  [(j :left index)
   (j :right index)])

;; queries on collections of joints: should we
;;   a) not allow
;;   b) return a sequence of speeds
;;   c) return average of speeds?

(defn on-ground?
  "tests whether the head or a leg (extending out from given joint) is
   contacting the ground."
  ([]
     (let [cset (contacting (:head *it*))]
       (cset @ground-body)))
  ([jt]
     (let [cset (contacting (body-b jt))]
       (cset @ground-body))))

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
    (dorun (map #(j-world-angle! % ang speed-factor) jt))
    (do
      (enable-motor! jt true)
      (let [ang-diff (in-pi-pi (- ang (angle (body-b jt))))]
        (motor-speed! jt (* speed-factor ang-diff))))))

(defn j-maxtorque!
  [jt torque]
  (if (coll? jt)
    (dorun (map #(j-maxtorque! % torque) jt))
    (max-motor-torque! jt torque)))

(defn lay-egg!
  ([]
     (lay-egg! @energy))
  ([energy-invested]
     (swap! energy-in-eggs + energy-invested)
     (swap! energy - energy-invested)))
