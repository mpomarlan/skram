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

(defclass task-context ()
  ((location-resolvers :initarg :location-resolvers :initform nil :reader location-resolvers)
   (task :initarg :task :initform nil :reader task)))

(defclass execution-context ()
  ((scene-setup :initarg :scene-setup :initform nil :reader scene-setup)
   (tasks :initarg :tasks :initform nil :reader tasks)
   (prolog-query :initarg :prolog-query :initform nil :reader prolog-query)))

(defun init-projection ()
  (prolog:def-fact-group costmap-metadata ()
    (prolog:<- (location-costmap:costmap-size 12 12))
    (prolog:<- (location-costmap:costmap-origin -6 -6))
    (prolog:<- (location-costmap:costmap-resolution 0.04))
    (prolog:<- (location-costmap:costmap-padding 0.3))
    (prolog:<- (location-costmap:costmap-manipulation-padding 0.35))
    (prolog:<- (location-costmap:costmap-in-reach-distance 0.55))
    (prolog:<- (location-costmap:costmap-reach-minimal-distance 0.2))
    (prolog:<- (location-costmap:visibility-costmap-size 2))
    (prolog:<- (location-costmap:orientation-samples 2))
    (prolog:<- (location-costmap:orientation-sample-step 0.1)))

  (setf cram-bullet-reasoning-belief-state:*robot-parameter* "robot_description")
  (setf cram-bullet-reasoning-belief-state:*kitchen-parameter* "kitchen_description")

  (cram-occasions-events:clear-belief)
  (setf cram-tf:*tf-default-timeout* 2.0)
  (setf prolog:*break-on-lisp-errors* t)
  (cram-bullet-reasoning:clear-costmap-vis-object)
  (setf cram-tf:*tf-broadcasting-enabled* t)
  (setf pr2-proj-reasoning::*projection-reasoning-enabled* nil)
  (setf ccl::*is-client-connected* nil)
  (setf ccl::*is-logging-enabled* nil)
  (setf ccl::*host* "'https://192.168.100.172'")
  (setf ccl::*cert-path* "'/home/ease/openease-certificates/sebastian.pem'")
  (setf ccl::*api-key* "'hftn9KwE77FEhDv9k6jV7rJT7AK6nPizZJUhjw5Olbxb2a3INUL8AM3DNp9Ci6L1'")
  (ccl::connect-to-cloud-logger)
  (ccl::reset-logged-owl))

(roslisp-utilities:register-ros-init-function init-projection)

(cpl-impl:def-cram-function execute-context-internal (execution-context)
  (cpl-impl:with-failure-handling
    ((cpl-impl:plan-failure (e)
      (declare (ignore e))
      (return)))
    (prolog:prolog '(btr:clear-bullet-world))
    (cram-occasions-events:clear-belief)
    (mapcar (lambda (obj-desig)
              (add-object-at-designator btr:*current-bullet-world* obj-desig))
            (scene-setup execution-context))
    (mapcar (lambda (task)
              (mapcar #'desig:enable-location-generator-function (location-resolvers task))
              (pp-plans::park-arms)
              (prolog:prolog '(and (btr:bullet-world ?world)
                                   (btr:simulate ?world 4)))
              (exe:perform
                (desig:a motion (type moving-torso) (joint-angle 0.3)))
              (exe:perform (task task))
              (mapcar #'desig:disable-location-generator-function (location-resolvers task)))
            (tasks execution-context))))

(defun execute-context (execution-context)
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl-impl:top-level
      (execute-context-internal execution-context))))

