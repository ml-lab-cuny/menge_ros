/*

License

Menge
Copyright © and trademark ™ 2012-14 University of North Carolina at Chapel Hill. 
All rights reserved.

Permission to use, copy, modify, and distribute this software and its documentation 
for educational, research, and non-profit purposes, without fee, and without a 
written agreement is hereby granted, provided that the above copyright notice, 
this paragraph, and the following four paragraphs appear in all copies.

This software program and documentation are copyrighted by the University of North 
Carolina at Chapel Hill. The software program and documentation are supplied "as is," 
without any accompanying services from the University of North Carolina at Chapel 
Hill or the authors. The University of North Carolina at Chapel Hill and the 
authors do not warrant that the operation of the program will be uninterrupted 
or error-free. The end-user understands that the program was developed for research 
purposes and is advised not to rely exclusively on the program for any reason.

IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE AUTHORS 
BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL 
DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS 
DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE 
AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY 
DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY STATUTORY WARRANTY 
OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND 
THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS HAVE NO OBLIGATIONS 
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Any questions or comments should be sent to the authors {menge,geom}@cs.unc.edu

*/

/*!
 *	@file		NullViewer.h
 *	@brief		Specification for a do-nothing viewer.  This is the
 *				offline simulator.
 */

#ifndef __NULL_VIEWER_H__
#define __NULL_VIEWER_H__

#include "CoreConfig.h"
#include "Profiler.h"

// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <std_msgs/Bool.h>
#include "menge_core/RunSim.h"

namespace Menge {

	// forward declarations
	namespace SceneGraph {
		class GLScene;
	}

	namespace Vis {

		/*!
		 *	@brief		The off-line context for running a simulation.
		 *
		 *				The NullViewer is a mock viewer, similar to the GLViewer
		 *				It takes a SceneGraph node and repeatedly updates the scene
		 *				However, there is no visualization.  It's purpose is simply
		 *				to run the scene.
		 *
		 *				The POINT of this type of thing is to load up an empty scene
		 *				with a non-trivial system that has some secondary, non-visual
		 *				side effects and simply exercise it.
		 */
		class MENGE_API NullViewer {
		public:
			/*!
			 *	@brief		Default constructor.
			 */
			NullViewer();

			/*!
			 *	@brief		Destructor.
			 */
			~NullViewer();

			/*!
			 *	@brief		The scene to run.
			 */
			void setScene( SceneGraph::GLScene * scene );

			/*!
			 *	@brief		The main loop.
			 *
			 *	@param      &queue          ROS callback queue handling the control of the viewer
			 */
			void run(ros::CallbackQueue &queue);

			/*!
			 *	@brief		Sets the simulator to use a fixed time step, with the given value.
			 *
			 *	@param		stepSize		The size of the fixed step the viewer should advance
			 *								its GLScene.
			 */
			void setFixedStep( float stepSize );

            /*!
             *	@brief		Add ROS node handle to NullViewer
             *
             *	@param		pointer to node handle
             */

            void setStepFromMsg(const std_msgs::Bool::ConstPtr& msg);

            void setRunFromMsg(const std_msgs::Bool::ConstPtr& msg);

            bool setStepFromSrv(menge_core::RunSim::Request &req, menge_core::RunSim::Response &res);

            void addNodeHandle( ros::NodeHandle *nh, ros::CallbackQueue &queue){
                _nh = nh;
                _nh->setCallbackQueue(&queue);
                _sub_step = _nh->subscribe("step", 1000, &Menge::Vis::NullViewer::setStepFromMsg, this);
                _sub_run = _nh->subscribe("run", 1000, &Menge::Vis::NullViewer::setRunFromMsg, this);
                _srv_run = _nh->advertiseService("advance_simulation", &Menge::Vis::NullViewer::setStepFromSrv, this);
                _pub_done = _nh->advertise<std_msgs::Bool>("done", 1);
                _spinner.reset(new ros::AsyncSpinner(0, &queue));
            }
            /*!
             *	@brief		return ROS node handle
             *
             *	@param		void
             */
            ros::NodeHandle* getNodeHandle(){return _nh;}

		protected:
			/*!
			 *	@brief		The GLScene to draw.
			 */
			SceneGraph::GLScene *	_scene;

			/*!
			 *	@brief		The step size for fixed-step simulation.
			 */
			float	_stepSize;

			/*!
			 *	@brief		Timer for determining computation time.
			 */
			LapTimer	_fpsTimer;

            /*!
            *	@brief		Indicates whether the viewer scene has been updated or not
            */
            bool    _scene_updated;

            /*!
             *	@brief		Determines if simulation steps are requested via ROS service
             */
            bool    _srv_run_received;

            /*!
             *	@brief		number of simulation steps requested via ROS service
             */
            int     _srv_num_steps;

            /*!
             *	@brief		_viewTime when service was received
             */
            float    _srv_start_time;

            /*!
             *	@brief		Controls whether the viewer advances the GLScene (true) or not (false).
             */
            bool	_pause;

            /*!
             *	@brief		Determines if a simulation step is requested via ROS message
             */
            bool    _step;

            /*!
			 *	@brief		The current time at which the viewer is running.  Modified by calls to GLViewer::setTime and
			 *				GLViewer::offsetTime.
			 */
            float _viewTime;

            /*!
             *	@brief		ROS node handle
             */

            ros::NodeHandle *_nh;
            ros::Subscriber _sub_step;
            ros::Subscriber _sub_run;
            ros::ServiceServer _srv_run;
            ros::Publisher _pub_done;
            boost::shared_ptr<ros::AsyncSpinner> _spinner;
		};
	}	// namespace Vis
}	// namespace Menge
#endif	// __NULL_VIEWER_H__
