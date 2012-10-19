
* creature movement strategies
  * debug/insight:
    * press \? to show API state
    * count how often and for how long each operator is active
  * decision tree (DONE)
    * effects at each node, not just leaves
    * abandoned in favour of:
  * sets of conditions proposing operators (like SOAR)
    * DONE

* energy
  * food particles
    * food -> mass (density) -> energy
    * touch food with head to eat
  * account for energy use
    * depletes reserves DONE
    * physiological max energy use rate?
      * over some time span
      * depends on food type?
  * lay eggs (using energy)
    * reproductive success
    * energy in egg gives starting reserve for offspring
    * egg sits for a time (vulnerable) before hatching
  * account for damage to head *********
    * as energy loss?

* creature senses
  * positions / angles / velocity
  * integrated velocity (over some time span)
  * center of mass vs legs
    * on/off balance
    * weight on/off a given leg
  * motor speeds
  * contacts
  * AABB query for food
    * several ranges / directions
  * sensors/raycast?

* evolution
  * crossover and mutation
    * mutation rules evolve?
  * some balance of competing individuals, or independent (islands)
    * predation!

* initially just one body structure
* other body structures?
  * evolve
  * generalised movement strategies?
    * need abstractions

* allow creatures to decide whether to pass through things or not
  * but some things are not passable
  * both creatures must agree to pass each other

