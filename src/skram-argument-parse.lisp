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

(defun resolvable-location (?location)
  (cpl-impl:with-failure-handling
    ((desig:designator-error (e)
      (declare (ignore e))
      (cpl-impl::return nil)))
    (desig:reference ?location)))

;;(object-at-location ?instrument-designator ?instrument-target-location)
;;(object-at-location ?instrument-designator (desig:a location (near ?location) (for ?instrument-designator)))
;;(split-objects-by-location-designator (?actees (desig:a location (on ?instrument-designator))))
;;(split-objects-by-location-designator objects ?actee-source)
;; check whether object is a designator or a btr:object
;; cast to a default type
;; location can be:
;;   - specific: contains :pose
;;       in this case just check that the object pose is the same/close enough to the :pose
;;   - non-specific:
;;       with :near property
;;         get a relatum: either the object designator that's the value for :near, or the object designator that's the value :for in the :near location
;;         SHOULD identify a(n indirect) support of the relatum, and check whether the object is itself supported by it
;;         instead, for now will simply apply distance thresholds
;;       no :near property
;;         get a relatum: an object that's the prop-value for :from, :at, :on, :in
;;         check that the object is supported by the relatum
(defun object-at-location (?object ?location)
  ;;TODO: need a more general function to handle designators here. For now, just use:
  ;;    near, but not too near;
  ;;    "from": several attempts eventually, for now just means "on", aka support;
  ;;    support.
  (let* ((?object (if (typep ?object 'desig:object-designator)
                    (bullet-object btr:*current-bullet-world* (desig:desig-prop-value ?object :name))
                    ?object))
         (?object-pose (bullet-pose ?object))
         (?location-pose (desig:desig-prop-value ?location :pose))
         (?near (desig:desig-prop-value ?location :near)))
    (if ?location-pose
      ;; will compare poses. NOTE: should check that they are in the same reference frame, for now we assume they are
      (< (cl-tf:v-dist (cl-tf:origin ?location-pose) (cl-tf:origin ?object-pose)) 0.05)
      (if ?near
        (let* ((?relatum (if (typep ?near 'desig:object-designator)
                           ?near
                           (desig:desig-prop-value ?near :for)))
               (?relatum (if (desig:desig-prop-value ?relatum :name)
                           ?relatum
                           (desig:reference ?relatum :add-name)))
               (?relatum-pose (bullet-pose (bullet-object btr:*current-bullet-world* (desig:desig-prop-value ?relatum :name)))))
          (< (cl-tf:v-dist (cl-tf:origin ?relatum-pose) (cl-tf:origin ?object-pose)) 0.05))
        (let* ((?relatum (or (desig:desig-prop-value ?location :on)
                             (desig:desig-prop-value ?location :in)
                             (desig:desig-prop-value ?location :at)
                             (desig:desig-prop-value ?location :from)))
               (?relatum (if (desig:desig-prop-value ?relatum :name)
                           ?relatum
                           (desig:reference ?relatum :add-name)))
               (?relatum (bullet-object btr:*current-bullet-world* (desig:desig-prop-value ?relatum :name)))
               (?supported (btr::list-supported-objects btr:*current-bullet-world* ?relatum)))
          (member ?object ?supported))))))

(defun split-objects-by-location-designator (?objects ?location &optional ?objects-at-location ?objects-elsewhere)
  (if ?objects
    (let* ((?object (car ?objects))
           (?objects (cdr ?objects)))
      (if (object-at-location ?object ?location)
        (split-objects-by-location-designator ?objects ?location (cons ?object ?objects-at-location) ?objects-elsewhere)
        (split-objects-by-location-designator ?objects ?location ?objects-at-location (cons ?object ?objects-elsewhere))))
    (list ?objects-at-location ?objects-elsewhere)))

(defun localized-actee-at-source-internal (?actee-designator ?source-designator)
  (let* ((?actee-name (desig:desig-prop-value ?actee-designator :name))
         (?actee-type (desig:desig-prop-value ?actee-designator :type))
         (?actee-source (or (desig:desig-prop-value ?actee-designator :in)
                            (desig:desig-prop-value ?actee-designator :from)
                            (desig:desig-prop-value ?actee-designator :at)
                            (desig:desig-prop-value ?actee-designator :on)
                            ?source-designator))
         (objects (remove-if #'null
                             (mapcar (lambda (object)
                                       (if (or (and (not ?actee-name) (not ?actee-type))
                                               (and ?actee-name (equal ?actee-name (btr:name object)))
                                               (and (not ?actee-name) ?actee-type (typep object 'btr:item) (member ?actee-type (btr::item-types object))))
                                         object
                                         nil))
                               (btr:objects btr:*current-bullet-world*))))
         (objects (if (and ?actee-source (not ?actee-name))
                    (first (split-objects-by-location-designator objects ?actee-source))
                    objects))
         (objects (if (equal (desig:desig-prop-value ?actee-designator :quantity) :plural)
                    objects
                    (list (car objects)))))
    (mapcar (lambda (object)
              (let* ((?object-name (bullet-name object))
                     (?object (desig:make-designator :object
                                                     (desig:update-designator-properties `((:quantity :singular)
                                                                                           (:name ,?object-name))
                                                                                         (desig:properties ?actee-designator)))))
                (desig:reference ?object :add-name)))
            objects)))

(defun localized-actee-at-source (?actees ?source-designator)
  (let* ((?actees (if (listp ?actees) ?actees (list ?actees))))
    (apply #'append (mapcar (lambda (?actee)
                              (localized-actee-at-source-internal ?actee ?source-designator))
                            ?actees))))

(defun agent-link (semantic-description)
  ;; TODO: need to add some agent to skram plans, and then refer to the agent's description
  (declare (ignore semantic-description))
  "base_footprint")

;; TODO: robot puppetteering is a bit of a hack because multiple robots at the same time are not supported
(defun spawn-puppet-pr2 (id &optional (pose '((0 -5 0) (0 0 0 1))))
  (let* ((robot-urdf (cl-urdf:parse-urdf (roslisp:get-param "robot_description"))))
    (prolog:prolog `(and (btr:bullet-world ?w)
                         (assert (btr:object ?w :urdf ,id ,pose
                                             :urdf ,robot-urdf))))))

(defun get-robot-joint-states (robot-id)
  (let* ((robot (bullet-object btr:*current-bullet-world* robot-id)))
    (when robot
      (btr:joint-states robot))))

(defun set-robot-joint-states (robot-id joint-states)
  (let* ((robot (bullet-object btr:*current-bullet-world* robot-id)))
    (when robot
      (maphash (lambda (k v)
                 (setf (btr:joint-state robot k) v))
               joint-states))))

(defun puppetteer-pr2 (driver-pr2 puppet-pr2)
  (let* ((joint-states (get-robot-joint-states driver-pr2))
         (pose (cl-tf:ensure-pose-stamped (bullet-pose driver-pr2) "map" 0)))
    (set-robot-joint-states puppet-pr2 joint-states)
    (btr-utils:move-object driver-pr2 (cl-tf:make-pose-stamped "map" 0 (cl-tf:make-3d-vector 0 0 0) (cl-tf:euler->quaternion)))
    (btr-utils:move-object puppet-pr2 pose)))

