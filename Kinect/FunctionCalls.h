/***********************************************************************
FunctionCalls - Set of functor objects implementing function (or method)
calls as first-class variables.
Copyright (c) 2012 Oliver Kreylos

This file is part of the Kinect 3D Video Capture Project (Kinect).

The Kinect 3D Video Capture Project is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The Kinect 3D Video Capture Project is distributed in the hope that it
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Kinect 3D Video Capture Project; if not, write to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
**********************************************************************/

#ifndef KINECT_FUNCTIONCALLS_INCLUDED
#define KINECT_FUNCTIONCALLS_INCLUDED

#include <Misc/FunctionCalls.h>

namespace Misc {

/* Specialized abstract base class for function calls with no parameters: */
template <>
class FunctionCall<void>
	{
	/* Constructors and destructors: */
	public:
	virtual ~FunctionCall(void)
		{
		}
	
	/* Methods: */
	virtual void operator()(void) const =0; // Function call operator
	};

/* Class to call C-style functions: */
template <>
class VoidFunctionCall<void>:public FunctionCall<void>
	{
	/* Embedded classes: */
	public:
	typedef void (*Function)(void); // Type for function pointers
	
	/* Elements: */
	private:
	Function function; // The function pointer
	
	/* Constructors and destructors: */
	public:
	VoidFunctionCall(Function sFunction) // Creates a functor wrapper for the given function
		:function(sFunction)
		{
		}
	
	/* Methods from FunctionCall: */
	virtual void operator()(void) const
		{
		/* Simply call the function: */
		function();
		}
	};

/* Class to call C-style functions taking a single additional argument of arbitrary type: */
template <class ArgumentParam>
class SingleArgumentFunctionCall<void,ArgumentParam>:public FunctionCall<void>
	{
	/* Embedded classes: */
	public:
	typedef ArgumentParam Argument; // Argument type
	typedef void (*Function)(Argument); // Type for function pointers
	
	/* Elements: */
	private:
	Function function; // The function pointer
	Argument argument; // The argument to pass to the function
	
	/* Constructors and destructors: */
	public:
	SingleArgumentFunctionCall(Function sFunction,const Argument& sArgument) // Creates a functor wrapper for the given function and argument
		:function(sFunction),argument(sArgument)
		{
		}
	
	/* Methods from FunctionCall: */
	virtual void operator()(void) const
		{
		/* Call the function with the provided argument: */
		function(argument);
		}
	
	/* New methods: */
	void setArgument(const Argument& newArgument) // Changes the function call argument
		{
		argument=newArgument;
		}
	};

/* Class to call C++ methods: */
template <class CalleeParam>
class VoidMethodCall<void,CalleeParam>:public FunctionCall<void>
	{
	/* Embedded classes: */
	public:
	typedef CalleeParam Callee; // Type of called objects
	typedef void (Callee::*Method)(void); // Type for method pointers
	
	/* Elements: */
	private:
	Callee* callee; // Object whose method to call
	Method method; // The method pointer
	
	/* Constructors and destructors: */
	public:
	VoidMethodCall(Callee* sCallee,Method sMethod) // Creates a functor wrapper for the given method on the given object
		:callee(sCallee),method(sMethod)
		{
		}
	
	/* Methods from FunctionCall: */
	virtual void operator()(void) const
		{
		/* Call the method on the provided object: */
		(callee->*method)();
		}
	};

/* Class to call C++ methods on const objects: */
template <class CalleeParam>
class VoidConstMethodCall<void,CalleeParam>:public FunctionCall<void>
	{
	/* Embedded classes: */
	public:
	typedef CalleeParam Callee; // Type of called objects
	typedef void (Callee::*Method)(void) const; // Type for method pointers
	
	/* Elements: */
	private:
	const Callee* callee; // Object whose method to call
	Method method; // The method pointer
	
	/* Constructors and destructors: */
	public:
	VoidConstMethodCall(const Callee* sCallee,Method sMethod) // Creates a functor wrapper for the given method on the given object
		:callee(sCallee),method(sMethod)
		{
		}
	
	/* Methods from FunctionCall: */
	virtual void operator()(void) const
		{
		/* Call the method on the provided object: */
		(callee->*method)();
		}
	};

/* Class to call C++ methods taking a single additional argument of arbitrary type: */
template <class CalleeParam,class ArgumentParam>
class SingleArgumentMethodCall<void,CalleeParam,ArgumentParam>:public FunctionCall<void>
	{
	/* Embedded classes: */
	public:
	typedef CalleeParam Callee; // Type of called objects
	typedef ArgumentParam Argument; // Argument type
	typedef void (Callee::*Method)(Argument); // Type for method pointers
	
	/* Elements: */
	private:
	Callee* callee; // Object whose method to call
	Method method; // The method pointer
	Argument argument; // The argument to pass to the method
	
	/* Constructors and destructors: */
	public:
	SingleArgumentMethodCall(Callee* sCallee,Method sMethod,const Argument& sArgument) // Creates a functor wrapper for the given method on the given object and the given argument
		:callee(sCallee),method(sMethod),argument(sArgument)
		{
		}
	
	/* Methods from FunctionCall: */
	virtual void operator()(void) const
		{
		/* Call the method on the provided object with the provided argument: */
		(callee->*method)(argument);
		}
	
	/* New methods: */
	void setArgument(const Argument& newArgument) // Changes the method call argument
		{
		argument=newArgument;
		}
	};

/* Class to call C++ methods taking a single additional argument of arbitrary type on const objects: */
template <class CalleeParam,class ArgumentParam>
class SingleArgumentConstMethodCall<void,CalleeParam,ArgumentParam>:public FunctionCall<void>
	{
	/* Embedded classes: */
	public:
	typedef CalleeParam Callee; // Type of called objects
	typedef ArgumentParam Argument; // Argument type
	typedef void (Callee::*Method)(Argument) const; // Type for method pointers
	
	/* Elements: */
	private:
	const Callee* callee; // Object whose method to call
	Method method; // The method pointer
	Argument argument; // The argument to pass to the method
	
	/* Constructors and destructors: */
	public:
	SingleArgumentConstMethodCall(const Callee* sCallee,Method sMethod,const Argument& sArgument) // Creates a functor wrapper for the given method on the given object and the given argument
		:callee(sCallee),method(sMethod),argument(sArgument)
		{
		}
	
	/* Methods from FunctionCall: */
	virtual void operator()(void) const
		{
		/* Call the method on the provided object with the provided argument: */
		(callee->*method)(argument);
		}
	
	/* New methods: */
	void setArgument(const Argument& newArgument) // Changes the method call argument
		{
		argument=newArgument;
		}
	};

/****************
Helper functions:
****************/

inline
FunctionCall<void>*
createFunctionCall(
	void (*function)(void))
	{
	return new VoidFunctionCall<void>(function);
	}

template <class ArgumentParam>
inline
FunctionCall<void>*
createFunctionCall(
	void (*function)(const ArgumentParam&),
	ArgumentParam argument)
	{
	return new SingleArgumentFunctionCall<void,const ArgumentParam&>(function,argument);
	}

template <class CalleeParam>
inline
FunctionCall<void>*
createFunctionCall(
	CalleeParam* callee,
	void (CalleeParam::*method)(void))
	{
	return new VoidMethodCall<void,CalleeParam>(callee,method);
	}

template <class CalleeParam>
inline
FunctionCall<void>*
createFunctionCall(
	const CalleeParam* callee,
	void (CalleeParam::*method)(void) const)
	{
	return new VoidConstMethodCall<void,CalleeParam>(callee,method);
	}

template <class CalleeParam,class ArgumentParam>
inline
FunctionCall<void>*
createFunctionCall(
	CalleeParam* callee,
	void (CalleeParam::*method)(const ArgumentParam&),
	ArgumentParam argument)
	{
	return new SingleArgumentMethodCall<void,CalleeParam,const ArgumentParam&>(callee,method,argument);
	}

template <class CalleeParam,class ArgumentParam>
inline
FunctionCall<void>*
createFunctionCall(
	const CalleeParam* callee,
	void (CalleeParam::*method)(const ArgumentParam&) const,
	ArgumentParam argument)
	{
	return new SingleArgumentConstMethodCall<void,CalleeParam,const ArgumentParam&>(callee,method,argument);
	}

}

#endif
