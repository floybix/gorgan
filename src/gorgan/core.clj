(ns gorgan.core
  (:use [cljbox2d core joints testbed])
  (:require [quil.core :as quil]))

(def it (atom {}))

(def walking (atom :right))

(def SPEED (/ PI 4))

(defn ff [body]
  (first (fixtureseq body)))

(defn setup-world! []
  (create-world!)
  (let [ground (body! (body-def :type :static)
                      (fixture-def (edge [-40 0] [40 0]))
                      (fixture-def (edge [-40 0] [-50 20]))
                      (fixture-def (edge [40 0] [50 20])))
        head (body! (body-def :position [0 3])
                    (fixture-def (circle 1) :density 5))
        thigh (body! (body-def :position (world-point head [2 0]))
                     (fixture-def (box 1 0.1)))
        calf (body! (body-def :position (world-point thigh [1 -1]))
                    (fixture-def (box 0.1 1) :friction 1))
        j1 (joint! (revolute-joint-def head thigh
                                       (world-point head [1 0])
                                       :lower-angle (/ (- PI) 4)
                                       :upper-angle (/ PI 4)
                                       :enable-limit true
                                       :enable-motor false
                                       :motor-speed SPEED
                                       :motor-torque 1000))
        j2 (joint! (revolute-joint-def thigh calf
                                       (world-point thigh [1 0])
                                       :lower-angle (/ (- PI) 4)
                                       :upper-angle (/ PI 4)
                                       :enable-limit true
                                       :enable-motor false
                                       :motor-speed SPEED
                                       :motor-torque 1000))]
    (reset! it {:head head
                :thigh thigh
                :calf calf
                :j1 j1
                :j2 j2})
    (reset! ground-body ground)))

(defn joint-angle
  [jt]
  (.getJointAngle jt))

(defn update-info-text []
  (let [j1 (:j1 @it)
        j2 (:j2 @it)]
  (reset! info-text
          (str "First movement experiment." "\n"
               "j1 angle = " (format "%.2f" (.getJointAngle j1))
               " speed = " (format "%.2f" (.getJointSpeed j1))
               " torque = " (.getMotorTorque j1) "\n"
               "j2 angle = " (format "%.2f" (.getJointAngle j2))
               " speed = " (format "%.2f" (.getJointSpeed j2))
               " torque = " (.getMotorTorque j2) "\n"))))

(defn setup []
  (setup-world!)
  (update-info-text))

(defn draw []
  (step! (/ 1 (quil/current-frame-rate)))
  (let [j1 (:j1 @it)
        j2 (:j2 @it)]
    (.enableMotor j1 true)
    (.enableMotor j2 true)
    (if (> (joint-angle j2) (- (/ PI 4) 0.1))
      (do
        (.setMotorSpeed j1 (- SPEED))
        (.setMotorSpeed j2 (- SPEED)))
      (if (< (joint-angle j2) (+ (/ (- PI) 4) 0.1))
        (do
          (.setMotorSpeed j1 SPEED)
          (.setMotorSpeed j2 SPEED)))))
  (update-info-text)  
  (draw-world))

(defn -main
  "Run the sketch."
  [& args]
  (quil/defsketch the-sketch
    :title "Gorgan"
    :setup setup
    :draw draw
    :mouse-pressed mouse-pressed
    :mouse-released mouse-released
    :mouse-dragged mouse-dragged
    :size [600 500]))
