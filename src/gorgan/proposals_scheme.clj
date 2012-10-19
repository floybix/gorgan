(ns gorgan.proposals-scheme
  (:use [cljbox2d.core :only [world-time]]
        [cljbox2d.testbed :only [*timestep*]]
        [gorgan.core :only [core-step *debug*
                            last-operator
                            prev-operator
                            last-priority
                            proposals
                            operator-chosen-at
                            operator-durations]])
  (:require [gorgan.dsl :as dsl]))

;; a movement scheme ruleset
(def ^:dynamic *scheme*)

(defn decide!
  "Returns a vector [operator-name priority]."
  [scheme]
  (reset! proposals {})
  ;; operator priority inertia: drops to zero in 3 seconds
  (let [elapsed (- @world-time @operator-chosen-at)
        frac (max 0 (- 1 (/ elapsed 3)))
        priority-inertia (* @last-priority frac)]
    (swap! proposals assoc @last-operator priority-inertia))
  (binding [*ns* (find-ns 'gorgan.dsl)]
    (eval (:productions scheme)))
  (if (seq @proposals)
    (apply max-key val @proposals)
    [nil 0]))

(defn step
  []
  (core-step)
  (let [[op priority] (decide! *scheme*)]
    (swap! operator-durations
           #(assoc % op (+ (get % op 0) *timestep*)))
    (when-not (= op @last-operator)
      (reset! operator-chosen-at @world-time)
      (reset! prev-operator @last-operator)
      (reset! last-operator op)
      (reset! last-priority priority))
    ;; apply the operator
    (binding [*ns* (find-ns 'gorgan.dsl)]
      (eval (get-in *scheme* [:operators @last-operator])))))

(def first-scheme
  {:productions
   '[(when-all [(> (food-within 1) 0)]
               (propose :crouch 100))
     (when-all [(> (food-left 10) 0)]
               (propose :walk-left (* (food-left 10) 2.0)))
     (when-all [(> (food-right 10) 0)]
               (propose :walk-right (* (food-right 10) 2.0)))
     (when-all [true]
               (propose :stand-up 1))
     (when-all [(> (time-in-state) 10)]
               (propose :escape 120))]
   :operators
   '{:crouch [(j-off! (j))]
     :walk-left [(j-maxtorque! (j) 500)
                 (j-speed! (j :left 0) 2)
                 (j-speed! (j :right 0) 2)
                 (j-world-angle! (j :left 1) 0 8)
                 (j-world-angle! (j :right 1) 0 8)]
     :walk-right [(j-speed! (j :left 0) -2)
                  (j-speed! (j :right 0) -2)
                  (j-world-angle! (j :left 1) 0 8)
                  (j-world-angle! (j :right 1) 0 8)]
     :stand-up [(j-world-angle! (j :left) 0 2)
                (j-world-angle! (j :right) 0 2)]
     :escape [(j-speed! (j :left) 3)]}
   })
