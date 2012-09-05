(ns gorgan.core
  (:use [cljbox2d core joints testbed]
        [cljbox2d.vec2d :only [TWOPI PI angle* angle-left?]])
  (:require [quil.core :as quil]))

(def it (atom {}))

(def SPEED (/ PI 4))

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
        ped (make-2ped [0 3] -2)]
    (reset! it ped)
    (reset! ground-body ground)))

(defn draw-info-text []
  (let [{{[l0 l1] :joints} :left
         {[r0 r1] :joints} :right} @it
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
         {[r0 r1] :joints} :right} @it
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
  (let [{{[l0 l1] :joints} :left
         {[r0 r1] :joints} :right} @it]
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
