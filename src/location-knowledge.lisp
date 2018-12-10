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

(defun interpret-spatial-relation (?designator &optional (role :manipulation-location))
  (cond
    ((equal role :manipulation-location)
      (cond
        ((or (desig:desig-prop-value ?designator :on) (desig:desig-prop-value ?designator :in))
          ?designator)
        ((or (desig:desig-prop-value ?designator :from) (desig:desig-prop-value ?designator :towards) (desig:desig-prop-value ?designator :at))
          (let* ((?relatum (or (desig:desig-prop-value ?designator :from) (desig:desig-prop-value ?designator :towards) (desig:desig-prop-value ?designator :at))))
            (desig:copy-designator ?designator :new-description `((:on , ?relatum)))))
        (T
          ?designator)))
    (T
      ?designator)))

(defun find-highest-object (?objects height &optional (highest nil))
  (let* ((?object (car ?objects))
         (?objects (cdr ?objects))
         (z (when ?object (cl-tf:z (bullet-pose ?object)))))
    (if ?object
      (if (< height z)
        (find-highest-object ?objects z ?object)
        (find-highest-object ?objects height highest))
      highest)))

(defun stacking-on (?desig)
  (let* ((?relatum (desig:desig-prop-value ?desig :on))
         (?relatum (when ?relatum (desig:reference ?relatum :add-name)))
         (?relatum (when ?relatum (bullet-object btr:*current-bullet-world* (desig:desig-prop-value ?relatum :name))))
         (?supported (when ?relatum (btr::list-supported-objects btr:*current-bullet-world* ?relatum)))
         (?locatum (desig:desig-prop-value ?desig :for))
         (?locatum (when ?locatum (desig:reference ?locatum :add-name)))
         (?locatum (when ?locatum (bullet-object btr:*current-bullet-world* (desig:desig-prop-value ?locatum :name))))
         (?locatum-types (when (and ?locatum (typep ?locatum 'btr:item)) (btr::item-types ?locatum)))
         (?supported (when (and ?locatum-types ?supported)
                       (mapcar (lambda (?object)
                                 (when (and (typep ?object 'btr:item) (intersection (btr::item-types ?object) ?locatum-types))
                                   ?object))
                               ?supported)))
         (?supported (remove-if #'null ?supported))
         (?tallest-supported (find-highest-object ?supported -10))
         (?tallest-supported-pose (when ?tallest-supported (bullet-pose ?tallest-supported)))
         (?retq-pose (when ?tallest-supported-pose
                       (cl-tf:make-pose-stamped "map" 0
                                                (cl-tf:v+ (cl-tf:origin ?tallest-supported-pose)
                                                          (cl-tf:z (btr::bounding-box-dimensions (btr:aabb ?tallest-supported))))
                                                (cl-tf:orientation ?tallest-supported-pose)))))
    ?retq-pose))

(desig:register-location-generator
 5 stacking-on
 "Returns a location to create a stack of similar objects")

;; By default, stacking-on is disabled. Some execution contexts might re-enable it though.
(desig:disable-location-generator-function 'stacking-on)

