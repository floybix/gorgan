(ns gorgan.core
  (:use [cljbox2d.core]
        [cljbox2d.joints :only [revolute-joint! power-watts]]
        [cljbox2d.testbed :only [*timestep* info-text]]
        [cljbox2d.vec2d :only [TWOPI PI in-pi-pi
                               angle* angle-left?]])
  (:require [gorgan.history-queues :as hq]
            [quil.core :as quil]))

;; a creature
(def ^:dynamic *it*)

;; TODO these should be components of *it* :  (defrecord)?

;; energy accounting, all in joules
(def energy (atom nil))
(def energy-moving (atom nil))
(def energy-eaten (atom nil))
(def energy-in-eggs (atom nil))

;; each of these stores a history of values using several queues
(def pos-history (atom nil))
(def energy-history (atom nil))
(def energy-eaten-history (atom nil))

(def foodlist (atom []))

(def ^:dynamic *debug* true)

(defn make-food
  "Creates and returns a food particle.
   :user-data of the fixture is a map `{:is-food? true}`
   :user-data of the body is also a map `{:is-food? true, :joules joules}"
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

(defn draw-more []
  (let [fmt (fn [x] (format "%.1f" (float x)))]
    (reset! info-text
            (str "energy: 300 - " (fmt @energy-moving)
                 " + " (fmt @energy-eaten)
                 " = " (fmt @energy) "\n")))
  ;; draw energy reserves as bar
  (quil/rect-mode :corner)
  (quil/fill (quil/color 128 128 128))
  (quil/rect (- (quil/width) 10) 0 10 (quil/height))
  (quil/fill (quil/color 128 255 128))
  (quil/rect (- (quil/width) 10) 0 10 (* (quil/height)
                                         (/ @energy 1000))))

(defn core-step []
  (hq/history-queue-step! pos-history (position (:head *it*)))
  (hq/history-queue-step! energy-history @energy)
  (hq/history-queue-step! energy-eaten-history @energy-eaten)
  ;; Work(joules) = torque * rotation-angle
  ;; note joules/second = watts
  (let [jj (concat (:joints (:left *it*)) (:joints (:right *it*)))
        each-power (map power-watts jj)
        joules (* (reduce + each-power) *timestep*)]
    (swap! energy - joules)
    (swap! energy-moving + joules))
  (doseq [other (contacting (:head *it*))
          :let [info (user-data other)]]
    (when (:is-food? info)
      (swap! energy + (:joules info))
      (swap! energy-eaten + (:joules info))
      (destroy! other))))
