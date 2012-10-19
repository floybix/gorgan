(ns gorgan.worlds.first
  (:use [cljbox2d core joints testbed]
        [gorgan core])
  (:require [gorgan.proposals-scheme :as psch]
            [gorgan.history-queues :as hq]
            [quil.core :as quil]))

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
                         (make-food [x 12] 100)))
        ped (make-2ped [0 3] -2)]
    (alter-var-root (var *it*) (fn [_] ped))
    (alter-var-root (var psch/*scheme*) (fn [_] psch/first-scheme))
    (reset! energy 300)
    (reset! energy-moving 0)
    (reset! energy-eaten 0)
    (reset! energy-in-eggs 0)
    (hq/reset-history-queue! pos-history (position (:head *it*)))
    (hq/reset-history-queue! energy-history @energy)
    (hq/reset-history-queue! energy-eaten-history @energy-eaten)
    (reset! foodlist allfood)
    (reset! ground-body ground)))

(defn setup []
  (quil/frame-rate (/ 1 *timestep*))
  (setup-world!)
  (reset! step-fn psch/step)
  (reset! draw-more-fn draw-more))

(defn -main
  "Run the sketch."
  [& args]
  (quil/defsketch the-sketch
    :title "Gorgan - first world"
    :setup setup
    :draw draw
    :key-typed key-press
    :mouse-pressed mouse-pressed
    :mouse-released mouse-released
    :mouse-dragged mouse-dragged
    :size [1200 800]))
