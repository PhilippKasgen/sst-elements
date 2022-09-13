/*
* Copyright (c) 2012-2013 ARM Limited
* All rights reserved
*
* The license below extends only to copyright in the software and shall
* not be construed as granting a license to any other intellectual
* property including but not limited to intellectual property relating
* to a hardware implementation of the functionality of the software
* licensed hereunder.  You may use the software subject to the license
* terms below provided that you ensure that this notice is replicated
* unmodified and in its entirety in all distributions of the software,
* modified or unmodified, in source code or in binary form.
*
* Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
* Copyright (c) 2009 Advanced Micro Devices, Inc.
* Copyright (c) 2022 IMEC, Leuven and Philipp Kaesgen, Vinay Kumar Baapanapalli Yadaiah
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met: redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer;
* redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution;
* neither the name of the copyright holders nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "CheckTable.h"

namespace SST {
namespace MemHierarchyTester {

CheckTable::CheckTable(Output* _output,
    int _num_writers,
    int _num_readers,
    int seed,
    MemHierarchyTester* _tester) :
    output(_output),
    m_num_writers(_num_writers),
    m_num_readers(_num_readers),
    m_tester_ptr(_tester) {
    Addr physical = 0;
    srand(seed);

    /*
     * These constants were adapted from gem5's Ruby Tester
     */
    const unsigned size1 = 32;
    const unsigned size2 = 100;
    const unsigned base_address = 1000;
    const unsigned stride1 = CHECK_SIZE;
    const unsigned stride2 = 256;
    readsIssued = 0;
    writesIssued = 0;
    flushsIssued = 0;

    stdMemHandlers = new StdMemHandler(this, output);

    if (m_tester_ptr->areAddressesTied()) {
      const unsigned totalChecks = size1 + size2 + size2;
      const unsigned checksPerCore = totalChecks / m_num_writers;
      const unsigned base_address2 = m_tester_ptr->cachelineSize() * 20;
      const unsigned stride3 = m_tester_ptr->cachelineSize();
      physical = base_address2;

      for (unsigned node_id = 0;
          node_id < static_cast<unsigned>(m_num_writers); node_id++) {
        for (; physical < base_address2 + (node_id + 1) * checksPerCore * stride3;
            physical += stride3) {
          addCheck(physical, node_id);
          output->verbose(CALL_INFO, 1, 0,
              "address %ld to be checked by node %d\n", physical, node_id);
        }
      }
    } else {
      // The first set is to get some false sharing
      for (physical = base_address;
          physical < base_address + size1 * stride1; physical += stride1) {
          // Setup linear addresses
          addCheck(physical);
      }

      output->verbose(CALL_INFO, 1, 0, "Adding cache conflict checks\n");
      // The next two sets are to get some limited false sharing and
      // cache conflicts
      for (physical = base_address;
          physical < base_address + size2 * stride2; physical += stride2) {
          // Setup linear addresses
          addCheck(physical);
      }

      output->verbose(CALL_INFO, 1, 0, "Adding cache conflict checks2\n");
      for (physical = base_address + CHECK_SIZE;
          physical < base_address + size2 * stride2; physical += stride2) {
          // Setup linear addresses
          addCheck(physical);
      }
    }
}

CheckTable::~CheckTable() {
    unsigned size = m_check_vector.size();
    for (unsigned i = 0; i < size; i++)
        delete m_check_vector[i];
}

void
CheckTable::addCheck(const Addr& address) {
    if (floorLog2(CHECK_SIZE) != 0) {
        if ((address & ((1 << CHECK_SIZE_BITS) - 1)) != 0) {
          output->fatal(CALL_INFO, 1, "Check not aligned");
        }
    }

    for (unsigned i = address; i < address + CHECK_SIZE; i++) {
      if (m_lookup_map.count(i)) {
        // A mapping for this byte already existed, discard the entire check
        return;
      }
    }

    output->verbose(CALL_INFO, 1, 0, "Adding check for address: %ld\n", address);

    Check* check_ptr = new Check(output, address, m_num_writers,
                                 m_num_readers, m_tester_ptr);
    for (unsigned i = address; i < address + CHECK_SIZE; i++) {
        // Insert it once per byte
        m_lookup_map[i] = check_ptr;
    }
    m_check_vector.push_back(check_ptr);
}

void
CheckTable::addCheck(const Addr& address, const unsigned& node_id) {
    if (floorLog2(CHECK_SIZE) != 0) {
        if ((address & ((1 << CHECK_SIZE_BITS) - 1)) != 0) {
          output->fatal(CALL_INFO, 1, "Check not aligned");
        }
    }

    output->verbose(CALL_INFO, 1, 0, "Adding check for address: %ld\n", address);

    Check* check_ptr = new Check(output, address, m_num_writers,
                        m_num_readers, node_id, m_tester_ptr);
    for (unsigned i = address; i < address + CHECK_SIZE; i++) {
        // Assign the same check to CHECK_SIZE bytes
        m_lookup_map[i] = check_ptr;
    }
    m_check_vector.push_back(check_ptr);
}

Check* CheckTable::getRandomCheck() {
    assert(m_check_vector.size() > 0);
    return m_check_vector[rand() % m_check_vector.size()];
}

Check* CheckTable::getCheck(const Addr& address) {
    output->verbose(CALL_INFO, 1, 0, "Looking for check by address: %ld\n", address);

    auto i = m_lookup_map.find(address);

    if (i == m_lookup_map.end())
        return nullptr;

    Check* check = i->second;
    assert(check != nullptr);
    return check;
}


void CheckTable::StdMemHandler::handle(Interfaces::StandardMem::ReadResp* rsp) {
  Check* check_ptr = cpu->getCheck(rsp->pAddr);
  cpu->output->verbose(CALL_INFO, 1, 0, " XX: %s\n", rsp->getString().c_str());
  check_ptr->performCallback(rsp);
  cpu->readsIssued++;
}

void CheckTable::StdMemHandler::handle(Interfaces::StandardMem::WriteResp* rsp) {
  Check* check_ptr = cpu->getCheck(rsp->pAddr);
  cpu->output->verbose(CALL_INFO, 1, 0, " XX: %s\n", rsp->getString().c_str());
  check_ptr->performCallback(rsp);
  cpu->writesIssued++;
}

void CheckTable::StdMemHandler::handle(Interfaces::StandardMem::FlushResp* rsp) {
  Check* check_ptr = cpu->getCheck(rsp->pAddr);
  cpu->output->verbose(CALL_INFO, 1, 0, " XX: %s success=%d\n", rsp->getString().c_str(), rsp->getSuccess()?1:0);
  check_ptr->initiate2();
  cpu->flushsIssued++;
}

void CheckTable::StdMemHandler::handle(Interfaces::StandardMem::InvNotify* rsp) {
    Check* check_ptr = cpu->getCheck(rsp->pAddr);
    cpu->output->verbose(CALL_INFO, 1, 0, " XX: %s\n", rsp->getString().c_str());
}


void CheckTable::handleResponse(Interfaces::StandardMem::Request* ev) {

  output->verbose(CALL_INFO, 1, 0, "Received response from memory\n");

  ev->handle(stdMemHandlers);

  delete ev; // not deleting in any of the handlers

}

void CheckTable::print() const {
  output->output("CheckTable \n- Reads issued: %" PRIu64 "\n", readsIssued);
  output->output("- Writes issued: %" PRIu64 "\n", writesIssued);
  output->output("- Flushs issued: %" PRIu64 "\n", flushsIssued);

  for (auto* check : m_check_vector) {
    check->print();
  }
}

}  // namespace MemHierarchyTester
}  // namespace SST
