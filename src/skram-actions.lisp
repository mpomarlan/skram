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

(defun safe-pick (?robot ?arm ?object)
  (let* ((?object-name (desig:desig-prop-value ?object :name))
         (?object-type (desig:desig-prop-value ?object :type))
         (?object-pose (bullet-pose (bullet-object btr:*current-bullet-world* ?object-name)))
         (object-handle-poses (object-handle-poses ?object-name ?object-type)))
    (cpl-impl:with-failure-handling
      ((common-fail:manipulation-pose-unreachable (e)
         (declare (ignore e))
         (pp-plans:park-arms)
         (cpl-impl:retry))
       (desig:designator-error (e)
         (setf object-handle-poses (cdr object-handle-poses))
         (unless object-handle-poses
           (cpl-impl:fail e))
         (pp-plans:park-arms)
         (cpl-impl:retry)))
      (let* ((?handle-pose (car object-handle-poses)))
        (format t "Moving to grab handle at pose ~a~%" ?handle-pose)
        (cpl-impl:with-failure-handling
          ((common-fail:navigation-pose-unreachable (e)
             (declare (ignore e))
             (cpl-impl:retry)))
          (let* ((?go-pose (desig:reference (desig:a location (reachable-for pr2) (arm ?arm) (location (desig:a location (pose ?handle-pose)))))))
            (exe:perform (desig:an action (type going) (target (desig:a location (pose ?go-pose)))))))
        )
      (format t "Looking toward pose ~a~%" ?object-pose)
      (let* ((?object (car (localized-actee-at-source ?object nil)))
             (dummy (exe:perform (desig:a motion
                                          (type looking)
                                          (direction forward))))
             (dummy (exe:perform (desig:a motion
                                          (type looking)
                                          (target (desig:a location (pose ?object-pose))))))
             (?perceived-object (pp-plans:perceive ?object)))
        (declare (ignore dummy))
        (exe:perform (desig:an action
                               (type picking-up)
                               (arm ?arm)
                               (object ?perceived-object))))))
    (let* ((?lt (cl-tf:make-pose-stamped "base_footprint" 0
                                         (cl-tf:make-3d-vector 0.4 0.4 1.5)
                                         (cl-tf:euler->quaternion)))
           (?rt (cl-tf:make-pose-stamped "base_footprint" 0
                                         (cl-tf:make-3d-vector 0.4 -0.4 1.5)
                                         (cl-tf:euler->quaternion))))
      (exe:perform (desig:a motion
                            (type moving-tcp)
                            (right-target (desig:a location (pose ?rt)))
                            (left-target (desig:a location (pose ?lt)))))))

(defun safe-place (?robot ?arm ?object ?destination)
  (let* ((?object-name (desig:desig-prop-value ?object :name))
         (?object-type (desig:desig-prop-value ?object :type)))
(format t "Trying to safely place ~a at ~a~%" ?object ?destination)
    (cpl-impl:with-failure-handling 
      ((common-fail:manipulation-pose-unreachable (e)
         (declare (ignore e))
         (cpl-impl:retry))
       (desig:designator-error (e)
         (declare (ignore e))
         (cpl-impl:retry)))
      (let* ((retries 10)
             (?placing-location (desig:copy-designator ?destination :new-description `((:for ,(desig:an object (type ?object-type) (name ?object-name))))))
             (dummy (format t "PLACING LOCATION ~a~%" ?placing-location))
             (?placing-pose (if (eql (desig:desig-prop-value ?object :type) :tray) (tray-on-location (desig:desig-prop-value ?destination :on)) (desig:reference ?placing-location)))
             (?placing-pose (cl-tf:make-pose-stamped (cl-tf:frame-id ?placing-pose) (cl-tf:stamp ?placing-pose)
                                                     (cl-tf:v+ (cl-tf:make-3d-vector 0 0 0.02) (cl-tf:origin ?placing-pose))
                                                     (cl-tf:orientation ?placing-pose))))
        (cpl-impl:with-failure-handling
          ((common-fail:navigation-pose-unreachable (e)
             (declare (ignore e))
             (setf retries (- retries 1))
             (if (< 0 retries)
               (cpl-impl:retry)
               (cpl-impl:fail 'desig:designator-error))))
          (let* ((?go-pose (desig:reference (desig:a location (reachable-for pr2) (arm ?arm) (location (desig:a location (pose ?placing-pose)))))))
            (exe:perform (desig:an action (type going) (target (desig:a location (pose ?go-pose)))))))

        (let* ((?placing-pose ?placing-pose)
               )
        (format t "Will now place at ~a~%" (cl-tf:transform-pose cram-tf:*transformer* :pose ?placing-pose :target-frame "map"))
          (exe:perform (desig:an action
                                 (type placing)
                                 (arm ?arm)
                                 (target (desig:a location (pose ?placing-pose)))))
        (prolog:prolog `(btr:simulate ,btr:*current-bullet-world* 10))
          )))))

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
      (let* ((?source-support (desig:desig-prop-value ?source-designator :on))
             (?instrument-target-location (desig:a location (on ?source-support) (for ?instrument-designator))))
        (format t "Have a location on instrument~%")
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
           (?actees-yet-to-place (cdr ?actees-yet-to-place))
           (?destination-support (desig:desig-prop-value ?destination-designator :on)))
      (unless (object-at-location ?instrument-designator (desig:a location (on ?destination-support)
                                                                           ;(near (desig:a location (pose ?location)))
                                                                           (for ?instrument-designator)))
        (schematic-transport ?instrument-designator nil (desig:a location (on ?destination-support)
                                                                          ;(near (desig:a location (pose ?location)))
                                                                          (for ?instrument-designator)) nil))
      (schematic-transport ?actee nil (desig:copy-designator ?destination-designator) nil)
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
                (let* ((?arm (cdr (assoc (desig:desig-prop-value ?actee :type) *obj-grasp-arm*)))
                       (?arm (if ?arm ?arm :right))
                       (?robot 'cram-pr2-description:pr2)
                       (?destination-designator (desig:copy-designator ?destination-designator :new-description `((:for ,?actee)))))
                  (safe-pick ?robot ?arm ?actee)
                  (safe-place ?robot ?arm ?actee ?destination-designator)
                  (pp-plans:park-arms)
                  ;;(exe:perform (desig:an action
                  ;;                       (type transporting)
                  ;;                       (object ?actee)
                  ;;                       (target ?destination-designator)))
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
              (let* ((?object-type (desig:desig-prop-value ?object :type)))
                (safe-pick ?robot :right ?object)
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
                (safe-place ?robot :right ?object ?support-destination-designator)
                (exe:perform
                  (desig:a motion (type moving-torso) (joint-angle 0.3)))
                (pp-plans:park-arms)
                ))
            ?objects)))

(defun get-random-sample (sample-lims sample-orig attempts)
  (let* ((x (- (random (* (cl-tf:x sample-lims) 2)) (cl-tf:x sample-lims)))
         (y (- (random (* (cl-tf:y sample-lims) 2)) (cl-tf:y sample-lims)))
         (z (- (random (* (cl-tf:z sample-lims) 2)) (cl-tf:z sample-lims)))
         (new-sample (cl-tf:v+ sample-orig (cl-tf:make-3d-vector x y z))))
    (btr-utils:move-object :ball-0 (cl-tf:make-pose-stamped "map" 0 new-sample (cl-tf:euler->quaternion)))
    (if (<= attempts 0)
      new-sample
      (if (prolog:prolog `(btr:contact ,btr:*current-bullet-world* :ball-0 ?x))
        (get-random-sample sample-lims sample-orig (- attempts 1))
        new-sample))))

(defun find-closest-sample (samples new-sample &optional closest-sample closest-d)
  (if samples
    (let* ((cr-sample (car samples))
           (samples (cdr samples))
           (d (cl-tf:v-dist cr-sample new-sample))
           (closest-sample (if (or (not closest-sample)
                                   (not closest-d)
                                   (< d closest-d))
                             cr-sample
                             closest-sample))
           (closest-d (if (or (not closest-d) (< d closest-d))
                        d
                        closest-d)))
      (find-closest-sample samples new-sample closest-sample closest-d))
    closest-sample))

(defun check-edge-internal (start dir steps &optional extended)
  (if (< 0 steps)
    (let* ((cr (cl-tf:v+ start (cl-tf:v* dir 0.015))))
      (btr-utils:move-object :ball-0 (cl-tf:make-pose-stamped "map" 0 cr (cl-tf:euler->quaternion)))
      (if (prolog:prolog `(btr:contact ,btr:*current-bullet-world* :ball-0 ?x))
        (list nil start extended)
        (check-edge-internal cr dir (- steps 1) T)))
    (list T start T)))

(defun check-edge (start end)
  (let* ((diff (cl-tf:v- end start))
         (dist (cl-tf:v-norm diff))
         (dir (cl-tf:v* diff (/ 1 dist)))
         (steps (floor (/ dist 0.015))))
    (check-edge-internal start dir steps)))

(defun infiltrate-internal (sample-lims sample-orig source-verts dest-verts attempts)
  (if (eql 0 attempts)
    nil
    (let* ((new-sample (get-random-sample sample-lims sample-orig 1000))
           (closest-src (find-closest-sample source-verts new-sample))
           (closest-dest (find-closest-sample dest-verts new-sample))
           (edge-check-source (check-edge closest-src new-sample))
           (edge-check-dest (check-edge closest-dest new-sample))
           (connects-src (first edge-check-source))
           (connects-dest (first edge-check-dest))
           (new-sample-src (second edge-check-source))
           (new-sample-dest (second edge-check-dest))
           (extended-src (third edge-check-source))
           (extended-dest (third edge-check-dest)))
      (if (and connects-src connects-dest)
        T
        (infiltrate-internal sample-lims sample-orig
                             (if extended-src (cons new-sample-src source-verts) source-verts)
                             (if extended-dest (cons new-sample-dest dest-verts) dest-verts)
                             (- attempts 1))))))

(cpl-impl:def-cram-function infiltrate (?target-designator)
  (let* ((?target-designator (desig:reference (desig:copy-designator ?target-designator) :add-name))
         (target-object-name (desig:desig-prop-value ?target-designator :name))
         (target-object (btr:object btr:*current-bullet-world* target-object-name))
         (destination-pose (btr:pose target-object))
         (bb-size (btr:calculate-bb-dims target-object))
         (source-pose (cl-tf:make-pose-stamped "map" 0
                                               (cl-tf:v+ (cl-tf:origin destination-pose)
                                                         (cl-tf:make-3d-vector 0 0 (cl-tf:z bb-size)))
                                               (cl-tf:euler->quaternion)))
         (source-verts (list (cl-tf:origin source-pose)))
         (dest-verts (list (cl-tf:origin destination-pose))))
    (btr:remove-object btr:*current-bullet-world* :ball-0)
    (prolog:prolog `(btr:simulate ,btr:*current-bullet-world* 5))
    (btr:add-object btr:*current-bullet-world* :sphere :ball-0 source-pose :radius 0.005 :mass 0.01 :color '(1 0 0))
    (unless (infiltrate-internal bb-size (cl-tf:origin destination-pose) source-verts dest-verts 1000)
      (btr:remove-object btr:*current-bullet-world* :ball-0))))

