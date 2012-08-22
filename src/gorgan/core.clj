(ns gorgan.core
  (:use [cljbox2d core joints testbed])
  (:require [quil.core :as quil]))

(def it (atom {}))

(def walking (atom :right))

(def SPEED (/ PI 4))

;; TODO: need fn for point on edge of a shape at given angle
;; TODO: maybe also a point furthest from a given other point

(defn make-leg
  [body pos side]
  (let [z (side {:left -1, :right 1})
        wpos (world-point body pos)
        thigh (body! {:position (map + wpos [z 0])}
                     {:shape (box 1 0.1)})
        calf (body! {:position (world-point thigh [z -1])}
                    {:shape (box 0.1 1)})
        j1 (revolute-joint! body thigh wpos
                            {:lower-angle (/ (- PI) 4)
                             :upper-angle (/ PI 4)
                             :enable-limit true
                             :enable-motor false
                             :motor-speed (* SPEED 1.25)
                             :motor-torque 1000})
        j2 (revolute-joint! thigh calf
                            (world-point thigh [z 0])
                            {:lower-angle (/ (- PI) 4)
                             :upper-angle (/ PI 4)
                             :enable-limit true
                             :enable-motor false
                             :motor-speed SPEED
                             :motor-torque 1000})]
    {:thigh thigh
     :calf calf
     :j1 j1
     :j2 j2}))

(defn setup-world! []
  (create-world!)
  (let [ground (body! {:type :static}
                      {:shape (edge [-40 0] [40 0])}
                      {:shape (edge [-40 0] [-50 20])}
                      {:shape (edge [40 0] [50 20])})
        head (body! {:position [0 3]}
                    {:shape (circle 1) :density 5})
        rleg (make-leg head [1 0] :right)
        lleg (make-leg head [-1 0] :left)]
    (reset! it {:rleg rleg :lleg lleg :head head})
    (reset! ground-body ground)))

(defn joint-angle
  [jt]
  (.getJointAngle jt))

(defn update-info-text []
  (let [j1 (:j1 (:rleg @it))
        j2 (:j2 (:rleg @it))]
    (reset! info-text
            (str "First movement experiment." "\n"
                 "j1 angle = " (format "%.2f" (.getJointAngle j1))
                 " speed = " (format "%.2f" (.getJointSpeed j1))
                 " torque = " (format "%.2f" (.getMotorTorque j1)) "\n"
                 "j2 angle = " (format "%.2f" (.getJointAngle j2))
                 " speed = " (format "%.2f" (.getJointSpeed j2))
                 " torque = " (format "%.2f" (.getMotorTorque j2)) "\n"))))

(defn setup []
  (setup-world!)
  (update-info-text))

(defn draw []
  (step! (/ 1 (quil/current-frame-rate)))
  (doseq [leg [:rleg :lleg]]
    (let [j1 (:j1 (leg @it))
          j2 (:j2 (leg @it))]
      (.enableMotor j1 true)
      (.enableMotor j2 true)
      (if (> (joint-angle j2) (- (/ PI 4) 0.1))
        (do
          (.setMotorSpeed j1 (- SPEED))
          (.setMotorSpeed j2 (- SPEED)))
        (if (< (joint-angle j2) (+ (/ (- PI) 4) 0.1))
          (do
            (.setMotorSpeed j1 SPEED)
            (.setMotorSpeed j2 SPEED))))))
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
