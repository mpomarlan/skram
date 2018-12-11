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


(defparameter *test-pose-on-sink-table-1* (cl-tf:make-pose-stamped "map" 0 (cl-tf:make-3d-vector 1.42 0.6 0.863) (cl-tf:euler->quaternion)))
(defparameter *test-pose-on-sink-table-2* (cl-tf:make-pose-stamped "map" 0 (cl-tf:make-3d-vector 1.44 0.3 0.863) (cl-tf:euler->quaternion)))

(defparameter *test-table* (desig:make-designator :object `((:type :table) (:part-of :kitchen) (:urdf-name :kitchen-island-surface) (:quantity :singular))))
(defparameter *test-sink-table* (desig:make-designator :object `((:name (:kitchen :sink-area)) (:type :sink-table) (:part-of :kitchen) (:urdf-name :sink-area) (:quantity :singular))))
(defparameter *test-location-on-table* (desig:make-designator :location `((:on ,*test-table*) (:side :right))))
(defparameter *test-location-on-sink-table* (desig:make-designator :location `((:on ,*test-sink-table*))))
(defparameter *test-location-on-sink-table-1* (desig:make-designator :location `((:pose ,*test-pose-on-sink-table-1*))))
(defparameter *test-location-on-sink-table-2* (desig:make-designator :location `((:pose ,*test-pose-on-sink-table-2*))))
(defparameter *test-plate-on-table-1* (desig:make-designator :object `((:type :plate) (:at ,(desig:copy-designator *test-location-on-table*)))))
(defparameter *test-plate-on-table-2* (desig:make-designator :object `((:type :plate) (:at ,(desig:copy-designator *test-location-on-table*)))))
(defparameter *test-plate-on-sink-table-1* (desig:make-designator :object `((:type :plate) (:at ,(desig:copy-designator *test-location-on-sink-table-1*)))))
(defparameter *test-plate-on-sink-table-2* (desig:make-designator :object `((:type :plate) (:at ,(desig:copy-designator *test-location-on-sink-table-2*)))))
(defparameter *test-cup-on-sink-table-1* (desig:make-designator :object `((:type :cup) (:at ,(desig:copy-designator *test-location-on-sink-table-1*)))))
(defparameter *test-cup-on-sink-table-2* (desig:make-designator :object `((:type :cup) (:at ,(desig:copy-designator *test-location-on-sink-table-2*)))))


(defparameter *test-eating* (make-instance 'execution-context
                                           :scene-setup (list *test-plate-on-table-1*
                                                              *test-plate-on-table-2*)
                                           :tasks (list (make-instance 'task-context
                                                                       :location-resolvers nil
                                                                       :task (desig:make-designator
                                                                               :action
                                                                               `((:type :ingesting)
                                                                                 (:objects ,(desig:make-designator :object '((:type :plate) (:quantity :plural))))
                                                                                 (:source ,*test-location-on-table*)
                                                                                 (:instrument nil)))))))

(defparameter *test-washing* (make-instance 'execution-context
                                            :scene-setup (list *test-plate-on-sink-table-1*
                                                               *test-plate-on-sink-table-2*)
                                            :tasks (list (make-instance 'task-context
                                                                        :location-resolvers nil
                                                                        :task (desig:make-designator
                                                                                :action
                                                                                `((:type :material-removal)
                                                                                  (:material nil)
                                                                                  (:objects ,(desig:make-designator :object '((:type :plate) (:quantity :plural))))
                                                                                  (:source ,*test-location-on-sink-table*)
                                                                                  (:instrument nil)
                                                                                  (:support-destination ,*test-location-on-table*)
                                                                                  (:material-destination nil)))))))

(defparameter *test-transporting* (make-instance 'execution-context
                                                 :scene-setup (list *test-cup-on-sink-table-1*
                                                                    *test-cup-on-sink-table-2*)
                                                 :tasks (list (make-instance 'task-context
                                                                             :location-resolvers nil
                                                                             :task (desig:make-designator
                                                                                     :action
                                                                                     `((:type :schematic-transport)
                                                                                       (:objects ,(desig:make-designator :object '((:type :cup) (:quantity :plural))))
                                                                                       (:source ,*test-location-on-sink-table*)
                                                                                       (:instrument nil)
                                                                                       (:destination ,*test-location-on-table*)))))))

(defparameter *test-transporting-tray* (make-instance 'execution-context
                                                 :scene-setup (list *test-cup-on-sink-table-1*
                                                                    *test-cup-on-sink-table-2*)
                                                 :tasks (list (make-instance 'task-context
                                                                             :location-resolvers nil
                                                                             :task (desig:make-designator
                                                                                     :action
                                                                                     `((:type :schematic-transport)
                                                                                       (:objects ,(desig:make-designator :object '((:type :cup) (:quantity :plural))))
                                                                                       (:source ,*test-location-on-sink-table*)
                                                                                       (:instrument nil)
                                                                                       (:destination ,*test-location-on-table*)))))))

