(ns gorgan.history-queues
  (:use [cljbox2d.core :only [world-time]]
        [cljbox2d.testbed :only [*timestep*]]))

;; maintain a history of values with high resolution over a short time
;; span, down to lower resolutions at longer time spans. uses a
;; staggered series of queues:
;; * we keep one second, i.e. (/ 1 *timestep*) values in :steps
;; * every 1 second the oldest :steps is added to :secs
;; * we keep 60 seconds in :secs ... etc

;; a history-queue must be stored and passed as an atom.

(defn reset-history-queue!
  [hq val]
  (reset! hq
          (let [q (clojure.lang.PersistentQueue/EMPTY)]
            {:steps (into q (repeat (/ 1 *timestep*) val))
             :secs (into q (repeat 60 val))
             :mins (conj q val)
             :last-sec @world-time
             :last-min @world-time})))

(defn val-at-time-ago
  [hq tspan]
  (let [{:keys [steps secs mins]} @hq]
    (cond
     (< tspan 1) (nth steps (int (/ tspan *timestep*)))
     (< tspan 60) (nth secs (dec (int tspan)))
     :else (nth mins (dec (min (count mins) (int (/ tspan 60))))))))

(defn history-queue-step!
  [hq val]
  (swap! hq
         (fn [{:keys [steps secs mins last-sec last-min], :as m}]
           (let [next-min? (> @world-time (+ last-sec 60))
                 next-sec? (> @world-time (+ last-sec 1))]
             (merge m
                    {:steps (conj (pop steps) val)}
                    (when next-min?
                      {:mins (conj mins (peek secs))
                       :last-min @world-time})
                    (when next-sec?
                      {:secs (conj (pop secs) (peek steps))
                       :last-sec @world-time}))))))
