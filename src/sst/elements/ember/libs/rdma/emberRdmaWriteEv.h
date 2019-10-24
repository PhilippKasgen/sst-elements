// Copyright 2009-2018 NTESS. Under the terms
// of Contract DE-NA0003525 with NTESS, the U.S.
// Government retains certain rights in this software.
//
// Copyright (c) 2009-2018, NTESS
// All rights reserved.
//
// Portions are copyright of other developers:
// See the file CONTRIBUTORS.TXT in the top level directory
// the distribution for more information.
//
// This file is part of the SST software package. For license
// information, see the LICENSE file in the top level directory of the
// distribution.


#ifndef _H_EMBER_LIBS_RDMA_WRITE_EVENT
#define _H_EMBER_LIBS_RDMA_WRITE_EVENT

#include "emberRdmaEvent.h"

namespace SST {
namespace Ember {

class EmberRdmaWriteEvent : public EmberRdmaEvent {

public:
	EmberRdmaWriteEvent( RDMA::Interface& api, Output* output,
		   	RDMA::ProcAddr& procAddr, RDMA::Addr destAddr, Hermes::MemAddr& srcAddr, size_t length, int* retval ) :
		EmberRdmaEvent( api, output, retval ), procAddr(procAddr), destAddr(destAddr), srcAddr(srcAddr), length(length) {}

	~EmberRdmaWriteEvent() {}

    std::string getName() { return "RdmaWrite"; }

    void issue( uint64_t time, RDMA::Callback* callback ) {
        EmberEvent::issue( time );
        api.write( procAddr, destAddr, srcAddr, length, callback );
    }

private:
	RDMA::ProcAddr procAddr;
	RDMA::Addr destAddr;
   	Hermes::MemAddr srcAddr;
	size_t length;
};

}
}

#endif