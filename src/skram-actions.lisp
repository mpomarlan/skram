;;;
;;; Copyright (c) 2018, Mihai Pomarlan <pomarlan@uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :skram)

(cpl-impl:def-cram-function pick-located-object (?robot ?arm ?object)
  (let* ((?object-name (desig:desig-prop-value ?object :name))
         (?object-pose (bullet-pose (bullet-object btr:*current-bullet-world* ?object-name))))
    (cpl-impl:with-failure-handling
      ((common-fail:manipulation-pose-unreachable (e)
         (declare (ignore e))
         (cpl-impl:retry)))
      (btr-utils:move-object ?robot
                             (desig:reference (desig:a location (reachable-for ?robot)
                                                                (arm ?arm)
                                                                (location (desig:a location (pose ?object-pose))))))
      (let* ((?object (car (localized-actee-at-source ?object nil)))
             (dummy (exe:perform (desig:a motion
                                          (type looking)
                                          (target (desig:a location (pose ?object-pose))))))
             (?perceived-object (pp-plans:perceive ?object)))
        (declare (ignore dummy))
        (exe:perform (desig:an action
                               (type picking-up)
                               (arm ?arm)
                               (object ?perceived-object)))))))

(cpl-impl:def-cram-function place-object-at-location (?robot ?arm ?object ?destination)
  (let* ((?object-name (desig:desig-prop-value ?object :name))
    (cpl-impl:with-failure-handling 
      ((common-fail:manipulation-pose-unreachable (e)
         (declare (ignore e))
         (cpl-impl:retry))
       (desig:designator-error (e)
         (declare (ignore e))
         (cpl-impl:retry)))
      (let* ((?placing-location (desig:copy-designator ?destination :new-description `((:for ,(desig:an object (type ?object-type))))))
             (?placing-pose (desig:reference ?placing-location))
             (z-padding 0.04)
             (?placing-pose (cl-tf:make-pose-stamped (cl-tf:frame-id ?placing-pose) (cl-tf:stamp ?placing-pose)
                                                     (cl-tf:v+ (cl-tf:make-3d-vector 0 0 z-padding) (cl-tf:origin ?placing-pose))
                                                     (cl-tf:orientation ?placing-pose))))
             (btr-utils:move-object ?robot
                                    (desig:reference (desig:a location (reachable-for ?robot)
                                                                       (arm ?arm)
                                                                       (location (desig:a location (pose ?placing-pose))))))
             (let* ((?placing-pose (map-pose->footprint-pose ?placing-pose))
                    (?placing-pose (cl-tf:make-pose-stamped (cl-tf:frame-id ?placing-pose) (cl-tf:stamp ?placing-pose)
                                                            (cl-tf:origin ?placing-pose)
                                                            (cl-tf:euler->quaternion))))
               (exe:perform (desig:an action
                                      (type placing)
                                      (arm ?arm)
                                      (target (desig:a location (pose ?placing-pose))))))))))

        ;; ground instrument and objects to transport, build-up a list
        ;; split list into: objects already on the instrument, objects yet to place there[, objects already transported]
        ;; while already-on-instrument and yet-to-place,
        ;;   test for space on instrument:
        ;;     Y: check that instrument is at a location such that the free location is reachable, and so is the object yet to pick
        ;;       Y: grab the object, place it on the instrument
        ;;       N: bring the instrument near the object to pick
        ;;     N: check that instrument is at a location such that it is reachable, but so is a target location
        ;;       Y: grab an object, place it at the target location
        ;;       N: bring the instrument near to the target location

(cpl-impl:def-cram-function schematic-gather (?actees-yet-to-gather ?actees-yet-to-place ?source-designator ?destination-designator ?instrument-designator)
  (let* ((?actee (car ?actees-yet-to-gather))
         (?actees-yet-to-gather (cdr ?actees-yet-to-gather))
         (?location-on-instrument (when ?actee
                                    (desig:a location (for ?actee) (on ?instrument-designator)))))
    (if (and ?location-on-instrument (resolvable-location ?location-on-instrument))
      (let* ((?instrument-target-location (desig:a location (near ?actee) (for ?instrument-designator))))
        (unless (object-at-location ?instrument-designator ?instrument-target-location)
          (schematic-transport ?instrument-designator nil ?instrument-target-location nil))
        (schematic-transport ?actee ?source-designator (desig:a location (on ?instrument-designator) (for ?actee)) nil)
        (schematic-gather ?actees-yet-to-gather (cons ?actee ?actees-yet-to-place) ?source-designator ?destination-designator ?instrument-designator))
      (when ?actees-yet-to-place
        (schematic-place ?actees-yet-to-gather ?actees-yet-to-place ?source-designator ?destination-designator ?instrument-designator)))))

(cpl-impl:def-cram-function schematic-place (?actees-yet-to-gather ?actees-yet-to-place ?source-designator ?destination-designator ?instrument-designator)
  (if ?actees-yet-to-place
    (let* ((?actee (car ?actees-yet-to-place))
           (?location (desig:copy-designator ?destination-designator :new-description `((:for ,?actee))))
           (?location (desig:reference ?location))
           (?actees-yet-to-place (cdr ?actees-yet-to-place)))
      (unless (object-at-location ?instrument-designator (desig:a location (near ?location) (for ?instrument-designator)))
        (schematic-transport ?instrument-designator nil (desig:a location (near ?location) (for ?instrument-designator)) nil))
      (schematic-transport ?actee nil ?location nil)
      (schematic-place ?actees-yet-to-gather ?actees-yet-to-place ?source-designator ?destination-designator ?instrument-designator))
    (when ?actees-yet-to-gather
      (schematic-gather ?actees-yet-to-gather ?actees-yet-to-place ?source-designator ?destination-designator ?instrument-designator))))

(cpl-impl:def-cram-function schematic-transport (?actee-designator ?source-designator ?destination-designator ?instrument-designator)
               ;;(desig:an action
               ;;          (type transporting)
               ;;          (object ?object-to-fetch)
               ;;          ;; (arm right)
               ;;          (location ?fetching-location)
               ;;          (target ?delivering-location))
               ;;(desig:a location
               ;;         (of-item-object (desig:an object (name ?object-name))))
               ;;(desig:a location
               ;;         (of ?object-designator))
               ;;(desig:a location
               ;;         (pose ?pose-stamped))
  (format t "SCHEMATIC-TRANSPORT for ~a from ~a to ~a using ~a~%" ?actee-designator ?source-designator ?destination-designator ?instrument-designator)
  (let* ((?source-designator (interpret-spatial-relation ?source-designator))
         (?destination-designator (interpret-spatial-relation ?destination-designator))
         (?actees (localized-actee-at-source ?actee-designator ?source-designator)))
    (if ?instrument-designator
      (let* ((?instrument-designator (desig:reference ?instrument-designator :add-name))
             (?actees (split-objects-by-location-designator ?actees (desig:a location (on ?instrument-designator))))
             (?actees-yet-to-gather (second ?actees))
             (?actees-yet-to-place (first ?actees)))
        (schematic-gather ?actees-yet-to-gather ?actees-yet-to-place ?source-designator ?destination-designator ?instrument-designator))
      (mapcar (lambda (?actee)
                (let* ((?destination-designator (desig:copy-designator ?destination-designator :new-description `((:for ,?actee))))
                       (?arm (cdr (assoc (desig:desig-prop-value ?actee :type) *obj-grasping-arm*)))
                       (?robot 'cram-pr2-description:pr2))
                  (pick-located-object ?robot ?arm ?actee)
                  (place-object-at-location ?robot ?arm ?actee ?destination-designator)
                  ;(exe:perform (desig:an action
                  ;                       (type transporting)
                  ;                       (object ?actee)
                  ;                       (target ?destination-designator)))
                  ))
              ?actees))))

(defun schematic-ingestion-internal (?container ?instrument-designator)
  (declare (ignore ?instrument-designator))
  (let* ((?pose (bullet-pose (bullet-object btr:*current-bullet-world* (desig:desig-prop-value ?container :name)))))
    (btr-utils:move-object 'cram-pr2-description:pr2
                           (desig:reference (desig:a location (reachable-for pr2)
                                                              (arm right)
                                                              (location (desig:a location (pose ?pose)))))))
  (let* ((?pose (desig:reference (desig:a location (on ?container))))
         (?pose (map-pose->footprint-pose ?pose))
         (?pose (cl-tf:make-pose-stamped (cl-tf:frame-id ?pose) (cl-tf:stamp ?pose)
                                         (cl-tf:v+ (cl-tf:origin ?pose) (cl-tf:make-3d-vector -0.05 0 0.07))
                                         (cl-tf:euler->quaternion :az 0.085 :ay (/ pi 16)))))
    (exe:perform (desig:a motion
                          (type moving-tcp)
                          (right-target (desig:a location
                                                 (pose ?pose)))))))

(cpl-impl:def-cram-function schematic-ingesting (?actee-designator ?source-designator ?instrument-designator)
  (format t "SCHEMATIC-INGESTING of ~a from ~a using ~a~%" ?actee-designator ?source-designator ?instrument-designator)
  ;; TODO: a bit of a cheat here because we cannot yet load several PR2s. As a result, this plan already assumes two agents, and implements a "puppeteering" of one by the other
  (let* ((?source-designator (interpret-spatial-relation ?source-designator))
         (containers (localized-actee-at-source ?actee-designator ?source-designator))
         (?container-1 (first containers))
         (?container-2 (second containers)))
    (spawn-puppet-pr2 :pr2-2)
    (schematic-ingestion-internal ?container-1 ?instrument-designator)
    (puppetteer-pr2 'cram-pr2-description:pr2 :pr2-2)
    (schematic-ingestion-internal ?container-2 ?instrument-designator)))

(cpl-impl:def-cram-function schematic-material-removal (?material-designator ?support-designator ?source-designator ?material-destination-designator ?support-destination-designator ?instrument-designator)
  (format t "SCHEMATIC-MATERIAL-REMOVAL of ~a on ~a from ~a with ~a, and put the material at ~a and the support at ~a" ?material-designator ?support-designator ?source-designator ?instrument-designator ?material-destination-designator ?support-destination-designator)
  (let* ((?source-designator (interpret-spatial-relation ?source-designator))
         (?support-destination-designator (interpret-spatial-relation ?support-destination-designator))
         (?material-destination-designator (interpret-spatial-relation ?material-destination-designator))
         (?robot 'cram-pr2-description:pr2)
         (?objects (localized-actee-at-source ?support-designator ?source-designator))
         (?actee-pose (cl-tf:make-pose-stamped (agent-link "arms-base") 0.0
                                               (cl-tf:make-3d-vector 0.55 0 1.3)
                                               (cl-tf:euler->quaternion :ax (/ pi 4))))
         (?wash-pose-1 (cl-tf:make-pose-stamped (agent-link "arms-base") 0.0
                                                (cl-tf:make-3d-vector 0.55 -0.025 1.35)
                                                (cl-tf:euler->quaternion :az (/ pi -2) :ay (/ pi 4))))
         (?wash-pose-2 (cl-tf:make-pose-stamped (agent-link "arms-base") 0.0
                                                (cl-tf:make-3d-vector 0.55 0.025 1.25) 
                                                (cl-tf:euler->quaternion :az (/ pi -2) :ay (/ pi 4)))))
    (mapcar (lambda (?object)
              (let* ((?object-name (desig:desig-prop-value ?object :name))
                     (?object-type (desig:desig-prop-value ?object :type))
                     (?object-pose (bullet-pose (bullet-object btr:*current-bullet-world* ?object-name))))
                (cpl-impl:with-failure-handling
                  ((common-fail:manipulation-pose-unreachable (e)
                     (declare (ignore e))
                     (cpl-impl:retry)))
                  (btr-utils:move-object ?robot
                                         (desig:reference (desig:a location (reachable-for pr2)
                                                                            (arm right)
                                                                            (location (desig:a location (pose ?object-pose))))))
                  (let* ((?object (car (localized-actee-at-source ?object nil)))
                         (dummy (exe:perform (desig:a motion
                                                      (type looking)
                                                      (target (desig:a location (pose ?object-pose))))))
                         (?perceived-object (pp-plans:perceive ?object)))
                    (declare (ignore dummy))
                    (exe:perform (desig:an action
                                           (type picking-up)
                                           (arm right)
                                           (object ?perceived-object)))))
                (exe:perform (desig:a motion
                                      (type moving-tcp)
                                      (right-target (desig:a location
                                                             (pose ?actee-pose)))))
                (mapcar (lambda (iteration)
                          (declare (ignore iteration))
                          (exe:perform (desig:a motion
                                                (type moving-tcp)
                                                (left-target (desig:a location
                                                                      (pose ?wash-pose-1)))))
                          (exe:perform (desig:a motion
                                                (type moving-tcp)
                                                (left-target (desig:a location
                                                                      (pose ?wash-pose-2))))))
                        (alexandria:iota 5))
                (pp-plans:park-arms :arm :left)
                (cpl-impl:with-failure-handling 
                  ((common-fail:manipulation-pose-unreachable (e)
                     (declare (ignore e))
                     (cpl-impl:retry))
                   (desig:designator-error (e)
                     (declare (ignore e))
                     (cpl-impl:retry)))
                  (let* ((?placing-location (desig:copy-designator ?support-destination-designator :new-description `((:for ,(desig:an object (type ?object-type))))))
                         (?placing-pose (desig:reference ?placing-location))
                         (?placing-pose (cl-tf:make-pose-stamped (cl-tf:frame-id ?placing-pose) (cl-tf:stamp ?placing-pose)
                                                                 (cl-tf:v+ (cl-tf:make-3d-vector 0 0 0.4) (cl-tf:origin ?placing-pose))
                                                                 (cl-tf:orientation ?placing-pose))))
                    (btr-utils:move-object ?robot
                                           (desig:reference (desig:a location (reachable-for pr2)
                                                                              (arm right)
                                                                              (location (desig:a location (pose ?placing-pose))))))
                    (let* ((?placing-pose (map-pose->footprint-pose ?placing-pose))
                           (?placing-pose (cl-tf:make-pose-stamped (cl-tf:frame-id ?placing-pose) (cl-tf:stamp ?placing-pose)
                                                                   (cl-tf:origin ?placing-pose)
                                                                   (cl-tf:euler->quaternion))))
                      (exe:perform (desig:an action
                                             (type placing)
                                             (arm right)
                                             (target (desig:a location (pose ?placing-pose))))))))
                (exe:perform
                  (desig:a motion (type moving-torso) (joint-angle 0.3)))
                (pp-plans:park-arms)
                ))
            ?objects)))

