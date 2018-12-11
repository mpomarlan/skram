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

(defparameter *obj-masses* '((:cup . 0.2) (:plate . 0.4)))
(defparameter *obj-colors* '((:cup . (1 0 0)) (:plate . (1 0 0))))
(defparameter *obj-grasp-types* '((:cup . :top) (:plate . :right-side)))

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
