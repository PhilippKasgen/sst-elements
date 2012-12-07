// Copyright 2012 Sandia Corporation. Under the terms
// of Contract DE-AC04-94AL85000 with Sandia Corporation, the U.S.
// Government retains certain rights in this software.
//
// Copyright (c) 2012, Sandia Corporation
// All rights reserved.
//
// This file is part of the SST software package. For license
// information, see the LICENSE file in the top level directory of the
// distribution.

#ifndef _TRIVIALCPU_H
#define _TRIVIALCPU_H

#include <sst/core/event.h>
#include <sst/core/sst_types.h>
#include <sst/core/component.h>
#include <sst/core/link.h>
#include <sst/core/timeConverter.h>

#include "memEvent.h"

namespace SST {
namespace MemHierarchy {

class trivialCPU : public SST::Component {
public:

	trivialCPU(SST::ComponentId_t id, SST::Component::Params_t& params);
	int Setup() {return 0;}
	int Finish() {
		printf("TrivialCPU Finished\n");
		return 0;
	}

private:
	trivialCPU();  // for serialization only
	trivialCPU(const trivialCPU&); // do not implement
	void operator=(const trivialCPU&); // do not implement

	void handleEvent( SST::Event *ev );
	virtual bool clockTic( SST::Cycle_t );

	int workPerCycle;
	int commFreq;
	uint32_t maxAddr;

	std::set<MemEvent::id_type> requests;

	SST::Link* mem_link;

	friend class boost::serialization::access;
	template<class Archive>
	void save(Archive & ar, const unsigned int version) const
	{
		ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Component);
		ar & BOOST_SERIALIZATION_NVP(workPerCycle);
		ar & BOOST_SERIALIZATION_NVP(commFreq);
		ar & BOOST_SERIALIZATION_NVP(maxAddr);
		ar & BOOST_SERIALIZATION_NVP(mem_link);
	}

	template<class Archive>
	void load(Archive & ar, const unsigned int version)
	{
		ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Component);
		ar & BOOST_SERIALIZATION_NVP(workPerCycle);
		ar & BOOST_SERIALIZATION_NVP(commFreq);
		ar & BOOST_SERIALIZATION_NVP(maxAddr);
		ar & BOOST_SERIALIZATION_NVP(mem_link);
		//resture links
		mem_link->setFunctor(new SST::Event::Handler<trivialCPU>(this,&trivialCPU::handleEvent));
	}

	BOOST_SERIALIZATION_SPLIT_MEMBER()

};

}
}
#endif /* _TRIVIALCPU_H */
