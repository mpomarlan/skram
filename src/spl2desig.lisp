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

(defparameter *wccg-url* "https://litmus.informatik.uni-bremen.de/openccg/parse")

(defun call-wccg (nl-statement)
  (let* ((cmd (format nil "curl --data \"~a\" ~a" nl-statement *wccg-url*)))
    (yason:parse (uiop:run-program cmd :output :string))))

(defun get-parses (wccg-response)
  (let* ((json-parses (gethash "json_parses" wccg-response)))
    (when json-parses)
      (alexandria:hash-table-alist json-parses)))

(defun update-config (name config variables-data)
  (let* ((old-config (gethash name variables-data)))
    (if (not old-config)
      (let* ((config-class (gethash "__class__" config))
             (config-type (gethash "type" config))
             (config-name (gethash "name" config))
             (config-roles (gethash "roles" config))
             (new-config (make-hash-table :test #'equal)))
        (setf (gethash "__class__" new-config) config-class)
        (setf (gethash "type" new-config) config-type)
        (setf (gethash "name" new-config) config-name)
        (setf (gethash "roles" new-config) config-roles)
        (setf (gethash name variables-data) new-config))
      (let* ((old-roles (gethash "roles" old-config))
             (roles (gethash "roles" config))
             (new-roles (append old-roles roles)))
        (setf (gethash "roles" old-config)
              new-roles)))
    variables-data))

(defun collect-variables-data-internal (configs variables-data)
  (if configs
    (let* ((config (car configs))
           (configs (cdr configs))
           (name (when config
                   (gethash "name" config)))
           (variables-data (if name
                             (update-config name config variables-data)
                             variables-data))
           (roles (when config
                    (gethash "roles" config)))
           (roles (mapcar (lambda (role)
                            (when role
                              (let* ((target (gethash "target" role)))
                                (if (and (hash-table-p target)
                                         (gethash "name" target))
                                  target
                                  nil))))
                          roles))
           (roles (remove-if #'null roles))
           (variables-data (if roles
                             (collect-variables-data-internal roles variables-data)
                             variables-data)))
      (collect-variables-data-internal configs variables-data))
    variables-data))

(defun collect-variables-data (parse)
  (let* ((variables-data (make-hash-table :test #'equal))
         (main-configurations (if (listp parse) parse (list parse))))
    (collect-variables-data-internal main-configurations variables-data)))

(defun get-var-role (var-name role-name variables-data)
  (when var-name
    (let* ((var-name (if (hash-table-p var-name)
                       (gethash "name" var-name)
                       var-name))
           (variable-data (gethash var-name variables-data)))
      (when variable-data
        (let* ((roles (gethash "roles" variable-data))
               (roles (mapcar (lambda (role)
                                (when (equal (gethash "type" role) role-name)
                                  role))
                              roles))
               (roles (mapcar (lambda (role)
                                (when role
                                 (gethash "target" role)))
                              roles))
               (roles (remove-if #'null roles)))
          (if (equal (length roles) 1)
            (car roles)
            roles))))))

(defun spl->obj-desig-internal (description variables-data)
  (let* ((object-type (when description
                        (gethash "type" description)))
         (object-quantity (get-var-role description "quant" variables-data))
         (?quantity (cond
                      ((equal object-quantity "singular") :singular)
                      ((equal object-quantity "plural") :plural)
                      (T nil)))
         (?object-type (cdr (assoc object-type *CCG-Object-Type->CRAM-Object-Type* :test #'equal)))
         (?owl-name (cdr (assoc ?object-type *owl-names*)))
         (?urdf-name (cdr (assoc ?object-type *urdf-names*)))
         (?part-of (cdr (assoc ?object-type *part-ofs*))))
    (desig:make-designator :object
                           `((:type ,?object-type)
                             (:quantity ,?quantity)
                             (:owl-name ,?owl-name)
                             (:urdf-name ,?urdf-name)
                             (:part-of ,?part-of)))))

(defun spl->obj-desig (description variables-data)
  (if (listp description)
    (mapcar (lambda (description-fragment)
              (spl->obj-desig-internal description-fragment variables-data))
            description)
    (spl->obj-desig-internal description variables-data)))

(defun spl->loc-desig-internal (description variables-data)
  (let* ((relatum (get-var-role description "gs-relatum" variables-data))
         (?relatum (spl->obj-desig relatum variables-data))
         (spatial-modality (get-var-role description "gs-hasSpatialModality" variables-data))
         (spatial-modality-type (when spatial-modality
                                  (gethash "type" spatial-modality)))
         (?spatial-modality (cdr (assoc spatial-modality-type *CCG-spatial-modality->CRAM-spatial-modality* :test #'equal))))
    (desig:make-designator :location
                           `((,?spatial-modality ,?relatum)))))

(defun spl->loc-desig (description variables-data)
  (if (listp description)
    (mapcar (lambda (description-fragment)
              (spl->loc-desig-internal description-fragment variables-data))
            description)
    (spl->loc-desig-internal description variables-data)))

(defun adm-transport-plan (config-name variables-data)
  (let* ((actee (get-var-role config-name "gum-actee" variables-data))
         (route (get-var-role config-name "gs-route" variables-data))
         (route-type (when route
                       (gethash "type" route)))
         (valid-route (equal route-type "gs-GeneralizedRoute"))
         (route (when route
                  (gethash "name" route)))
         (valid-route (and valid-route route))
         (destination (when valid-route
                        (get-var-role route "gs-destination" variables-data)))
         (source (when valid-route
                        (get-var-role route "gs-source" variables-data)))
         (?actee (spl->obj-desig actee variables-data))
         (?destination (spl->loc-desig destination variables-data))
         (?source (spl->loc-desig source variables-data)))
    (desig:make-designator :action
                           `((:type :schematic-transport)
                             (:objects ,?actee)
                             (:source ,?source)
                             (:instrument nil)
                             (:destination ,?destination)))))

(defun affecting-directed-motion-plan (affecting-directed-motion-name variables-data)
  (let* ((process-in-configuration (get-var-role affecting-directed-motion-name "gum-processInConfiguration" variables-data))
         (process-type (gethash "type" process-in-configuration)))
    (cond
      ((member process-type '("slm-Bringing" "slm-Moving" "slm-Placing" "slm-Taking" "slm-Putting") :test #'equal)
        (adm-transport-plan affecting-directed-motion-name variables-data))
      (T
        nil))))

(defun parse-to-plan (parse)
  (if (not (listp parse))
    (let* ((variables-data (collect-variables-data parse))
           (config-type (gethash "type" parse))
           (config-name (gethash "name" parse)))
      (cond
        ((equal config-type "gs-AffectingDirectedMotion")
          (affecting-directed-motion-plan config-name variables-data))
        (T nil)))))

(defun interpret-nl (nl-statement)
  (let* ((server-response (call-wccg nl-statement))
         (parses (mapcar (lambda (named-parse)
                           (let* ((parses (cdr named-parse))
                                  (parse (if (equal (length parses) 1) (car parses) parses)))
                             parse))
                         (get-parses server-response))))
    (mapcar #'parse-to-plan parses)))

