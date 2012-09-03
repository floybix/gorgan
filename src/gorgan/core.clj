(ns gorgan.core
  (:use [cljbox2d core joints testbed]
        [cljbox2d.vec2d :only [TWOPI PI]])
  (:require [quil.core :as quil]))

(def it (atom {}))

(def walking (atom :right))

(def SPEED (/ PI 4))


(defn make-leg
  [body pos side]
  (let [z (side {:left -1, :right 1})
        wpos (position body pos)
        thigh (body! {:position (map + wpos [z 0])}
                     {:shape (box 1 0.1)})
        calf (body! {:position (position thigh [z -1])}
                    {:shape (box 0.1 1)})
        jt-opts {:lower-angle (/ (- PI) 4)
                 :upper-angle (/ PI 4)
                 :enable-limit true
                 :enable-motor false
                 :motor-speed SPEED
                 :motor-torque 100}
        j1 (revolute-joint! body thigh wpos
                            jt-opts)
        j2 (revolute-joint! thigh calf
                            (position thigh [z 0])
                            jt-opts)]
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

(defn draw-info-text []
  (let [{:keys [rleg lleg]} @it
        r1 (:j1 rleg)
        r2 (:j2 rleg)
        l1 (:j1 lleg)
        l2 (:j2 lleg)
        fmt (fn [x] (format "%.2f" x))
        j-info (fn [j nm]
                 (str nm
                      ": angle = " (fmt (joint-angle j))
                      ", motor " (if (motor-enabled? j) "on" "off")
                      ", mspeed = " (fmt (motor-speed j))
                      ", mtorque = " (fmt (motor-torque j))))]
    (quil/text-align :left)
    (quil/text (str (j-info l1 "l1") "\n"
                    (j-info l2 "l2") "\n"
                    (j-info r1 "r1") "\n"
                    (j-info r2 "r2"))
               10 10)))

(defn my-step []
  (doseq [leg [:rleg :lleg]]
    (let [j1 (:j1 (leg @it))
          j2 (:j2 (leg @it))]
      (enable-motor! j1 true)
      (enable-motor! j2 true)
      (if (> (joint-angle j2) (- (/ PI 4) 0.1))
        (do
          (motor-speed! j1 (- SPEED))
          (motor-speed! j2 (- SPEED)))
        (if (< (joint-angle j2) (+ (/ (- PI) 4) 0.1))
          (do
            (motor-speed! j1 SPEED)
            (motor-speed! j2 SPEED)))))))

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
    :key-typed key-press
    :mouse-pressed mouse-pressed
    :mouse-released mouse-released
    :mouse-dragged mouse-dragged
    :size [600 500]))
