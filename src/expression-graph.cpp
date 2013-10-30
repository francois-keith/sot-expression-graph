//#include "sampling_learned_models.h"
#include "expression-graph.h"
#include <dynamic-graph/all-commands.h>

//#include <ros/ros.h>

using namespace dynamicgraph::sotHri;
using namespace dynamicgraph;

#include <dynamic-graph/factory.h>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ExpressionGraph,"ExpressionGraph");

ExpressionGraph::
ExpressionGraph( const std::string& name )
	: Entity(name)
	//input signals
	, positionSIN(NULL,"ExpressionGraph("+name+")::input(matrixHomo)::position")

	//output signals
	, aSampleSOUT( boost::bind(&ExpressionGraph::sampleComputation,this,_1,_2),
				positionSIN,
				"ExpressionGraph("+name+")::output(vector)::gmrPosition" )
{
	// only register the external signals
	signalRegistration(positionSIN << aSampleSOUT);

	initCommands();
}

ExpressionGraph::~ExpressionGraph()
{
}


void ExpressionGraph::
initCommands()
{
	namespace dc = dynamicgraph::command;
//	addCommand("NameOfTheCommandInPythonInterface",
//	 dc::makeCommandVoid1(*this, &ExpressionGraph::callbackMethod,
//	 "description"));
}




ml::Vector&
ExpressionGraph::sampleComputation(ml::Vector& vec, int t)
{
  // get the value of the input signal
	const dg::sot::MatrixHomogeneous & position = positionSIN(t);

	vec.resize(1);
  vec(0) = 1;
	return vec;
}

