(ns gorgan.tree-scheme
  (:use [cljbox2d.core :only [world-time]]
        [gorgan.core :only [core-step *debug*]]
        [gorgan.dsl :only [*decision-path* last-decisions
                           entered-decisions]]))

;; a movement scheme tree
(def ^:dynamic *scheme*)

(defn decide!
  ([scheme]
     (decide! scheme []))
  ([scheme path]
     (binding [*ns* (find-ns 'gorgan.dsl)
               *decision-path* path]
       (when-let [actions (:do scheme)]
         (eval actions))
       (if-let [test-form (:test scheme)]
         (let [result (boolean (eval test-form))
               next-scheme (if result
                             (:when-true scheme)
                             (:when-false scheme))]
           (if next-scheme
             (decide! next-scheme (conj path result))
             path))
         path))))

(defn path-of-tests
  [scheme path]
  (loop [s scheme
         p path
         tests []]
    (if s
      (let [res (first path)
            next-s (if res (:when-true s) (:when-false s))]
        (recur next-s (next path) (conj tests (:test s))))
      tests)))

(defn step
  []
  (core-step)
  (let [path (decide! *scheme*)]
    (when-not (= path @last-decisions)
      ;; TODO: use a hook function here
      (when *debug*
        (println (format "@%.1f" @world-time)
                 "new decision: " path)
        (let [tests (path-of-tests *scheme* path)
              nt (count tests)]
          (doseq [[i tst res] (map vector (range nt) tests path)]
            (println (apply str (repeat (* i 2) \ ))
                     (if res "T" "F") ":"
                     tst))))
      (let [n-same (count (take-while true? (map = path @last-decisions)))
            n-more (- (count path) n-same)]
        (reset! last-decisions path)
        (swap! entered-decisions
               (fn [this] (concat (take n-same this)
                                  (repeat n-more @world-time))))))))

(def first-basic-scheme
  {
   :do ['(j-maxtorque! (j) 500)]
   :test '(> (food-within 1) 0)
   :when-true
   {
    :do ['(j-off! (j))]
    :test '(> (time-in-state) 3)
    :when-true
    {
     :do ['(j-speed! (j :left) 3)]
     }
    }
   :when-false
   {
    :do ['(j-world-angle! (j :left 1) 0 5)
         '(j-world-angle! (j :right 1) 0 5)]
    :test '(> (food-left 10) 0)
    :when-true
    {
     :do ['(j-speed! (j :left 0) 2)
          '(j-speed! (j :right 0) 2)]
     :test '(> (time-in-state) 5)
     :when-true
     {
      :do ['(j-speed! (j :left 1) 2)
           '(j-speed! (j :right 1) 2)]
      }
     }
    :when-false
    {
     :test '(> (food-right 10) 0)
     :when-true
     {
      :do ['(j-speed! (j :left 0) -2)
           '(j-speed! (j :right 0) -2)]
      :test '(> (time-in-state) 5)
      :when-true
      {
       :do ['(j-speed! (j :left 1) -5)
            '(j-speed! (j :right 1) -5)]
       }
      }
     :when-false
     {
      :do ['(j-world-angle! (j :left 0) 0 2)
           '(j-world-angle! (j :right 0) 0 2)]
      :test '(> (time-in-state) 2)
      :when-true
      {
       :do ['(j-speed! (j :left 0) (min-abs-limit (head-angle) 0.2))
            '(j-speed! (j :right 0) (min-abs-limit (head-angle) 0.2))]
       }
      }
     }
    }
   }
  )
