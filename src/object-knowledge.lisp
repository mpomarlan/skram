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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(defparameter *part-ofs* '((:sink-table . :kitchen)
                           (:table . :kitchen)))
(defparameter *urdf-names* '((:sink-table . :sink-area-surface)
                             (:table . :kitchen-island-surface)))
(defparameter *owl-names* nil)

(defparameter *obj-masses* '((:pot . 1.0) (:spatula . 0.4) (:fork . 0.1) (:cup . 0.2) (:plate . 0.4) (:tray . 0.6)))
(defparameter *obj-colors* '((:pot . (1 0 0)) (:spatula . (1 0 0)) (:fork . (1 0 0)) (:cup . (1 0 0)) (:plate . (1 0 0)) (:tray . (1 0 0))))
(defparameter *obj-grasp-types* '((:cup . :top) (:plate . :right-side) (:tray . :top)))
(defparameter *obj-grasp-arm* '((:tray . :right)))
(defparameter *obj-type-grasps* '((:tray (:right :top)) (:cup (:right :top) (:left :top)) (:plate (:left :left-side) (:right :right-side))))

(defparameter *obj-type-height* '((:pot . 0.168325) (:plate . 0.027733) (:cup . 0.094399) (:tray . 0.050063) (:fork . 0.026703)))

(defparameter *CCG-Object-Type->CRAM-Object-Type* '(("slm-Cup" . :cup)
                                                    ("slm-Table" . :table)
                                                    ("slm-Chair" . :chair)))

(defparameter *CCG-spatial-modality->CRAM-spatial-modality* '(("gum-FunctionalControlledExternal" . :on)
                                                              ("gum-FunctionalControlledInternal" . :in)
                                                              ("gs-GeneralDirectional" . :towards)
                                                              ("gs-PathRepresentingExternal" . :from)
                                                              ("gs-Proximal" . :near)))

(defun base-footprint-pose (&optional (robot 'cram-pr2-description:pr2))
  (btr:pose (gethash "base_footprint" (btr:links (btr:object btr:*current-bullet-world* robot)))))

(defun map-pose->footprint-pose (map-pose)
  (let* ((tr (cl-tf:transform* (cl-tf:transform-inv (cl-tf:pose->transform (base-footprint-pose)))
                               (cl-tf:pose->transform map-pose))))
    (cl-tf:make-pose-stamped "base_footprint" 0.0
                             (cl-tf:translation tr)
                             (cl-tf:rotation tr))))

(defun bullet-object (world object-name)
  (if (listp object-name)
    (let* ((whole (first object-name))
           (part (second object-name))
           (whole (btr:object world whole)))
      (when whole
        (gethash (roslisp-utilities:rosify-underscores-lisp-name part)
                 (btr:links whole))))
    (btr:object world object-name)))

(defun bullet-name (object)
  (cond
    ((typep object 'btr:rigid-body)
      (let* ((name (mapcar (lambda (s)
                             (roslisp-utilities:lispify-ros-underscore-name s :keyword))
                           (split-sequence:split-sequence #\. (format nil "~a" (btr:name object))))))
        (if (< 1 (length name))
          name
          (car name))))
    ((typep object 'btr:item)
      (btr:name object))
    (T nil)))

(defun bullet-pose (object)
  (cond
    ((equal object nil)
      nil)
    ((or (symbolp object) (keywordp object) (listp object))
      (bullet-pose (bullet-object btr:*current-bullet-world* object)))
    ((typep object 'btr:item)
      (cl-tf:ensure-pose-stamped (btr:pose object) "map" 0))
    ((typep object 'btr:robot-object)
      (cl-tf:ensure-pose-stamped (btr:pose (gethash "base_footprint" (btr:links object))) "map" 0))))

(defun object-handle-poses (object-name object-type)
  (let* ((object-location (bullet-pose object-name))
         (object-location (cl-tf:make-transform-stamped "map" "obj" 0
                                                        (cl-tf:origin object-location)
                                                        (cl-tf:orientation object-location)))
         (grasps (cdr (assoc object-type *obj-type-grasps*))))
    (cons 
      (bullet-pose object-name)
      (mapcar (lambda (grasp)
                (let* ((arm (first grasp))
                       (grasp (second grasp))
                       (obj-gr (cram-object-interfaces:get-object-type-to-gripper-transform object-type object-name arm grasp))
                       (handle-location (cl-tf:transform* object-location obj-gr)))
                  (cl-tf:make-pose-stamped "map" 0
                                           (cl-tf:translation handle-location)
                                           (cl-tf:rotation handle-location))))
              grasps))))

(defmethod desig:resolve-designator ((desig desig:object-designator) (role (eql :add-name)))
  (let* ((type (desig:desig-prop-value desig :type))
         (name (desig:desig-prop-value desig :name))
         (part-of (desig:desig-prop-value desig :part-of))
         (urdf-name (desig:desig-prop-value desig :urdf-name)))
    (cond
      (name 
        (if (listp name)
          (desig:copy-designator desig)
          (let* ((?object (desig:copy-designator desig))
                 (pose (bullet-pose (bullet-object btr:*current-bullet-world* name)))
                 (pose (map-pose->footprint-pose pose)))
            (with-slots ((data desig::data)) ?object
              (setf data (make-instance 'desig:object-designator-data :pose pose :object-identifier name)))
            (setf (gethash name btr-belief::*object-identifier-to-instance-mappings*) name)
            ?object)))
      ((and part-of urdf-name)
        (when (bullet-object btr:*current-bullet-world* `(,part-of ,urdf-name))
          (desig:copy-designator desig :new-description `((:name (,part-of ,urdf-name))))))
      (type
        (let* ((objects (remove-if #'null
                                   (mapcar (lambda (object)
                                             (when (and (typep object 'btr:item) (member type (btr::item-types object)))
                                               object))
                                           (btr:objects btr:*current-bullet-world*))))
               (object (car objects)))
          (when object
            (desig:reference (desig:copy-designator desig :new-description `((:name ,(bullet-name object)))) :add-name))))
      (T nil))))


(cram-object-interfaces:def-object-type-to-gripper-transforms :plate :right :left-side
  :grasp-translation `(0.0 ,(- kr-pp::*plate-grasp-y-offset*) ,kr-pp::*plate-grasp-z-offset*)
  :grasp-rot-matrix
  `((0             -1 0)
    (,(- (sin kr-pp::*plate-grasp-roll-offset*)) 0  ,(cos kr-pp::*plate-grasp-roll-offset*))
    (,(- (cos kr-pp::*plate-grasp-roll-offset*)) 0  ,(- (sin kr-pp::*plate-grasp-roll-offset*))))
  :pregrasp-offsets `(0.0 ,(- kr-pp::*plate-pregrasp-y-offset*) ,kr-pp::*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- kr-pp::*plate-pregrasp-y-offset*) ,kr-pp::*plate-2nd-pregrasp-z-offset*)
  :lift-offsets kr-pp::*lift-offset*
  :2nd-lift-offsets kr-pp::*lift-offset*)

(cram-object-interfaces:def-object-type-to-gripper-transforms :tray :right :right-side
  :grasp-translation `(0.0 ,(- kr-pp::*plate-grasp-y-offset*) ,kr-pp::*plate-grasp-z-offset*)
  :grasp-rot-matrix
  `((0             -1 0)
    (,(- (sin kr-pp::*plate-grasp-roll-offset*)) 0  ,(cos kr-pp::*plate-grasp-roll-offset*))
    (,(- (cos kr-pp::*plate-grasp-roll-offset*)) 0  ,(- (sin kr-pp::*plate-grasp-roll-offset*))))
  :pregrasp-offsets `(0.0 ,(- kr-pp::*plate-pregrasp-y-offset*) ,kr-pp::*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- kr-pp::*plate-pregrasp-y-offset*) ,kr-pp::*plate-2nd-pregrasp-z-offset*)
  :lift-offsets kr-pp::*lift-offset*
  :2nd-lift-offsets kr-pp::*lift-offset*)

(cram-object-interfaces:def-object-type-to-gripper-transforms :tray :right :top
  :grasp-translation `(0.0 0.2 0.035)
  :grasp-rot-matrix
  `(( 0 0 1)
    ( 0 1 0)
    ( -1 0 0))
  :pregrasp-offsets `(0.0 0.0 0.05)
  :2nd-pregrasp-offsets `(0.0 0.0 0.1)
  :lift-offsets kr-pp::*lift-offset*
  :2nd-lift-offsets kr-pp::*lift-offset*)

;(cram-object-interfaces:def-object-type-to-gripper-transforms :tray :left :top
;  :grasp-translation `(0.0 -0.2 0.035)
;  :grasp-rot-matrix
;  `(( 0 0 1)
;    ( 0 1 0)
;    ( -1 0 0))
;  :pregrasp-offsets `(0.0 0.0 0.05)
;  :2nd-pregrasp-offsets `(0.0 0.0 0.1)
;  :lift-offsets kr-pp::*lift-offset*
;  :2nd-lift-offsets kr-pp::*lift-offset*)

;(defmethod obj-int:get-object-type-to-gripper-transform ((object-type (eql :tray)) object-name (arm (eql :right))
;                                           (grasp (eql :right-side)))
;  (cl-tf:make-transform-stamped "" "" 0
;                                ()
;                                ()))

(defmethod obj-int:get-object-grasping-poses (object-name (object-type (eql :tray)) arm (grasp (eql :top)) object-transform)
    (declare (type symbol object-name object-type arm grasp)
             (type cl-transforms-stamped:transform-stamped object-transform))
    (let* ((object-to-left-handle (cl-tf:make-transform-stamped (cl-tf:frame-id object-transform) "left_handle" 0
                                                                (cl-tf:make-3d-vector -0.02 0.203 0.02)
                                                                (cl-tf:euler->quaternion)))
           (object-to-right-handle (cl-tf:make-transform-stamped (cl-tf:frame-id object-transform) "right_handle" 0
                                                                 (cl-tf:make-3d-vector -0.02 -0.203 0.02)
                                                                 (cl-tf:euler->quaternion)))
           (naive-footprint-to-left-handle (cl-tf:transform* object-transform object-to-left-handle))
           (naive-footprint-to-right-handle (cl-tf:transform* object-transform object-to-right-handle))
           (footprint-to-left-handle (if (< 0 (cl-tf:y (cl-tf:translation naive-footprint-to-left-handle))) naive-footprint-to-left-handle naive-footprint-to-right-handle))
           (footprint-to-right-handle (if (> 0 (cl-tf:y (cl-tf:translation naive-footprint-to-right-handle))) naive-footprint-to-right-handle naive-footprint-to-left-handle))
           (footprint-to-handle (ecase arm (:left footprint-to-left-handle) (:right footprint-to-right-handle)))
           (footprint-to-handle (cl-tf:copy-transform footprint-to-handle
                                                      :rotation (cl-transforms:make-identity-rotation)))
           (footprint-to-gripper (cl-tf:transform* footprint-to-handle (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0) (cl-tf:euler->quaternion))))
           (footprint-to-pregripper (cl-tf:transform* (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0.06) (cl-tf:euler->quaternion)) footprint-to-gripper))
           (footprint-to-pregripper2 (cl-tf:transform* (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0.12) (cl-tf:euler->quaternion)) footprint-to-gripper))
           )
      (mapcar (lambda (tr)
                (cl-tf:make-pose-stamped (cl-tf:frame-id object-transform) 0
                                         (cl-tf:translation tr)
                                         (cl-tf:rotation tr)))
              (list footprint-to-pregripper footprint-to-pregripper2 footprint-to-gripper footprint-to-pregripper2 footprint-to-pregripper))))


(defun tray-on-location (relatum)
  (let* ((relatum (desig:desig-prop-value relatum :urdf-name)))
    (ecase relatum
      (:table-area-main
        (cl-tf:make-pose-stamped "map" 0
                                 (cl-tf:make-3d-vector -1.7 -0.77 0.7327)
                                 (cl-tf:euler->quaternion :az (/ pi 2))))
      (:kitchen-island
        (cl-tf:make-pose-stamped "map" 0
                                 (cl-tf:make-3d-vector -0.65 1.5 0.94)
                                 (cl-tf:euler->quaternion)))
      (:kitchen-island-surface
        (cl-tf:make-pose-stamped "map" 0
                                 (cl-tf:make-3d-vector -0.65 1.5 0.94)
                                 (cl-tf:euler->quaternion)))
      (:sink-area
        (cl-tf:make-pose-stamped "map" 0
                                 (cl-tf:make-3d-vector 1.42 0.6 0.863)
                                 (cl-tf:euler->quaternion :az pi))))))

(prolog:def-fact-group tray-on-locations (desig:location-grounding)
  (prolog:<- (desig:location-grounding ?desig ?solution)
    (desig:desig-prop ?desig (:for ?for-object))
    (desig:desig-prop ?for-object (:type :tray))
    (desig:desig-prop ?desig (:on ?relatum))
    (prolog:lisp-fun tray-on-location ?relatum ?solution))
  )
