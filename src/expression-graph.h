/*
* Copyright 2010,
* François Bleibel,
* Olivier Stasse,
*
* CNRS/AIST
*
* This file is part of sot-hri.
* sot-hri is free software: you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public License
* as published by the Free Software Foundation, either version 3 of
* the License, or (at your option) any later version.
* sot-hri is distributed in the hope that it will be
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.  You should
* have received a copy of the GNU Lesser General Public License along
* with sot-hri.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __SOT_EXPRESSION_GRAPH_H__
#define __SOT_EXPRESSION_GRAPH_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <sot/core/flags.hh>
#include <sot/core/matrix-homogeneous.hh>

#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

/* STD */
#include <string>

namespace dynamicgraph { namespace sotHri {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*! \class ExpressionGraph
 \ingroup entities
 \brief
*/
class  ExpressionGraph
:public dg::Entity
{
public:
 /*! \brief Store the name of the class. */
 static const std::string CLASS_NAME;

 /*! \brief Returns the name class. */
 const std::string& getClassName( void ) const { return CLASS_NAME; }

public:
 /*! \brief Default constructor: the name of the class should be given. */
 ExpressionGraph( const std::string& name );
 ~ExpressionGraph();
 /*! \name Methods to control internal computation.
	 The main idea is that some feature may have a lower frequency
	 than the internal control loop. In this case, the methods for
	 computation are called only when needed.

	@{*/

 void initCommands();

 // the callback called at the computation.
 ml::Vector& sampleComputation(ml::Vector& vec, int t);

 /*! @} */

 /* --- SIGNALS ------------------------------------------------------------ */
public:

	// position of the end effector in the world frame
	dg::SignalPtr< dg::sot::MatrixHomogeneous,int > positionSIN;

	// force
	dg::SignalTimeDependent< ml::Vector,int > aSampleSOUT;

private:

 /*! @} */

 /*! @} */
};

} // namespace sot
} // namespace dynamicgraph


#endif

