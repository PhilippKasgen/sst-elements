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

#include <fstream>
#include <algorithm>
#include <vector>
#include <string>
#include <sst/core/interfaces/stdMem.h>

#include "Check.h"

using namespace SST::Interfaces;

namespace SST {

namespace MemHierarchyTester {

Check::Check(Output* _output,
    Addr address,
    int _num_writers,
    int _num_readers,
    MemHierarchyTester* _tester):
  output(_output),
  m_num_writers(_num_writers),
  m_num_readers(_num_readers),
  m_tester_ptr(_tester) {
    m_status = TesterStatus::Idle;

    pickValue();
    pickInitiatingNode();
    changeAddress(address);
}

Check::Check(Output* _output,
    Addr address,
    int _num_writers,
    int _num_readers,
    unsigned node_index_to_bind,
    MemHierarchyTester* _tester):
  output(_output),
  m_num_writers(_num_writers),
  m_num_readers(_num_readers),
  m_tester_ptr(_tester) {
    m_status = TesterStatus::Idle;

    m_initiatingNode = node_index_to_bind;
    m_store_count = 0;
    pickValue();
    changeAddress(address);
}

void Check::initiate() {
    // currently no protocols support prefetches
    if (false && ((rand() & 0xff) == 0)) {
        initiatePrefetch();  // Prefetch from random processor
    }

    bool flushNow = (rand() & 0xff) == 0;
    if (m_tester_ptr->getCheckFlush() && flushNow) {
        initiateFlush();  // issue a Flush request from random processor
    } else {
      initiate2();
    }
}

void Check::initiate2() {
  switch (m_status) {
    case TesterStatus::Idle: initiateAction(); break;
    case TesterStatus::Ready: initiateCheck(); break;
    default:
      output->verbose(CALL_INFO, 1, 0,
        "initiating action/check - failed: action/check is pending\n");
  }
}


void Check::initiateFlush() {
  unsigned index = m_tester_ptr->areAddressesTied() ? m_initiatingNode :
    rand() % m_num_writers;  // m_initiatingNode;
  SST::Interfaces::StandardMem* port = m_tester_ptr->getWritableCpuPort(index);

  m_tester_ptr->madeProgress(index);
  auto req = new Interfaces::StandardMem::FlushAddr(m_address, CHECK_SIZE, false, 10, 0, 0, 0, index); // FlushLine, not FlushLineInv

  port->send(req);

  if (output->getVerboseLevel() > 1) {
    std::string f_name = debug_file_name_prefix +
      std::to_string(m_initiatingNode) + debug_file_name_suffix;
    std::ofstream dump(f_name, std::ios::app);
    auto simTime = m_tester_ptr->getCurrentSimTime();

    dump << simTime << " FlushLine 0x" << std::hex << m_address
      << " 0x" << std::hex << CHECK_SIZE << std::endl;
    dump.close();
  }
}

void Check::initiateAction() {
  output->verbose(CALL_INFO, 1, 0, "Initiating Action\n");
  assert(m_status == TesterStatus::Idle);

  unsigned index = m_tester_ptr->areAddressesTied() ? m_initiatingNode :
    rand() % m_num_writers;  // m_initiatingNode;
  SST::Interfaces::StandardMem* port = m_tester_ptr->getWritableCpuPort(index);

  // Create the particular address for the next byte to be written
  Addr writeAddr(m_address + m_store_count);

  bool isRead = rand() & 1;

#if 1

  unsigned offset = writeAddr % m_tester_ptr->cachelineSize();
  uint32_t payload_size = (rand() % m_tester_ptr->payloadSize()) + 1;

  // shorten payload to avoid splitting
  if (offset + payload_size > m_tester_ptr->cachelineSize()) {
    //return; // TODOV skip this case
    payload_size = m_tester_ptr->cachelineSize() - offset;
  }
  SST::Interfaces::StandardMem::Request* req;
  if (isRead) {
    req = new SST::Interfaces::StandardMem::Read(writeAddr, payload_size, 0, writeAddr, 0, index); // TODOV thread ID?


  }
  else {

    if (m_store_count == 0) {
      m_tester_ptr->incrementChecksInPreparation();
    }

    std::vector<uint8_t> writeData(payload_size, 0);
    writeData[0] = m_value + m_store_count;
    for (unsigned i = 1; i < payload_size; i++) {
      writeData[i] = rand() & 0xff;
    }
    m_store_count++;
    if (writeAddr == 0x464) writeData[0] = 0x99;
    req = new StandardMem::Write(writeAddr, payload_size, writeData, false, 0, writeAddr, 0, index); // TODOV thread ID?
  output->verbose(CALL_INFO, 1, 0, "XXYY [ %" PRIu64 "] write %s\n", m_tester_ptr->getCurrentSimTime(), req->getString().c_str());

  }


  if (output->getVerboseLevel() > 1) {
    std::string f_name = debug_file_name_prefix
      + std::to_string(m_initiatingNode) + debug_file_name_suffix;
    std::ofstream dump(f_name, std::ios::app);
    auto simTime = m_tester_ptr->getCurrentSimTime();

    dump << simTime << " ";
    if (isRead) {
      dump << "Read 0x";
    }
    else {
      dump << "Write 0x";
    }
    dump << std::hex << writeAddr << " 0x" << std::hex << payload_size
      << std::endl;
    dump.close();
  }
  port->send(req);
#else
#endif

  m_status = TesterStatus::Action_Pending;
  m_tester_ptr->madeProgress(index);
}

void Check::initiateCheck() {
    output->verbose(CALL_INFO, 1, 0, "Initiating Check\n");
    assert(m_status == TesterStatus::Ready);

    m_tester_ptr->incrementChecksReady();

    unsigned index = m_tester_ptr->areAddressesTied() ? m_initiatingNode :
      rand() % m_num_writers;  // m_initiatingNode;
    Interfaces::StandardMem* port = m_tester_ptr->getReadableCpuPort(index);

#if 1
  auto req = new StandardMem::Read(m_address, CHECK_SIZE, 0, CHECK_SIZE, 0, index); // TODOV thread ID?

  port->send(req);

#else
#endif
    m_status = TesterStatus::Check_Pending;
    m_tester_ptr->madeProgress(index);

    output->verbose(CALL_INFO, 1, 0, "status after check update: %s\n",
        printCheckStatus(m_status));
}

const char* printCheckStatus(const TesterStatus& s) {
  switch (s) {
    case TesterStatus::Idle:           return "Idle"; break;
    case TesterStatus::Action_Pending: return "Action_Pending"; break;
    case TesterStatus::Ready:          return "Ready"; break;
    case TesterStatus::Check_Pending:  return "Check_Pending"; break;
    default:                return "Unknown__Please_seek_out_an_adult_for_help";
  }
}

Addr Check::makeLineAddress(const Addr& address) const {
  return address - (address % m_tester_ptr->cachelineSize());
}

void Check::performCallback(SST::Interfaces::StandardMem::ReadResp* resp) {
    SST::SimTime_t curTime = m_tester_ptr->getCurrentSimCycle();
    Addr address = resp->pAddr;

    assert(makeLineAddress(m_address) == makeLineAddress(address));

    output->verbose(CALL_INFO, 1, 0,
        "Callback for address %lx to check in state \"%s\"\n",
        resp->pAddr, printCheckStatus(m_status));

    switch (m_status) {
      case TesterStatus::Action_Pending:
        output->verbose(CALL_INFO, 1, 0,
            "Action callback write value: %d, currently %d\n",
            (m_value + m_store_count), resp->data[0]);

        m_status = m_store_count == CHECK_SIZE ?
          TesterStatus::Ready : TesterStatus::Idle;
        output->verbose(CALL_INFO, 1, 0,
            "Check %lx, State=%s\n", m_address, printCheckStatus(m_status));

        output->verbose(CALL_INFO, 1, 0, "Action callback return data now %d\n",
                resp->data[0]);
        break;

    // Check if the payload is equal to the expected value
      case TesterStatus::Check_Pending: {
          std::stringstream error;
          output->verbose(CALL_INFO, 1, 0, "Check callback %s\n", resp->getString().c_str());
          // Perform load/check
          if (m_tester_ptr->payloadSize() == 1) {
            for (unsigned byte_number = 0; byte_number < CHECK_SIZE; byte_number++) {
                output->verbose(CALL_INFO, 1, 0,
                    "Comparing previously written value with golden reference.\n");
                if (uint8_t(m_value + byte_number) != resp->data[byte_number]) {
                  error << "Action/check failure: proc: " << (resp->getID()) <<
                    " at address: " << std::hex << address << ", byte_number: " <<
                    byte_number << ", the expected value is " <<
                    static_cast<int>(m_value + byte_number) << " but received " <<
                    static_cast<int>(resp->data[byte_number]) <<
                    " instead. Time: " << std::dec << curTime << "\nTest failed"
                    << resp->getString() << std::endl;
                  output->verbose(CALL_INFO, 1, 0, error.str().c_str());
                  output->fatal(CALL_INFO, 1, error.str().c_str());
                }
            }
          }

          output->verbose(CALL_INFO, 1, 0, "Action/check success\n");

          m_tester_ptr->incrementCheckCompletions();

          m_status = TesterStatus::Idle;
          output->verbose(CALL_INFO, 1, 0, "Check %lx, State=Idle\n", m_address);
          pickValue();
        }
        break;

      default: {
          std::stringstream error;
          error << "Unexpected TesterStatus: " << (*this) << " proc: " <<
            resp << " data: ";
          for (auto&& d : resp->data) error << d;
          error << " m_status: " << printCheckStatus(m_status) <<
            " time: " << curTime << "\n";
          output->verbose(CALL_INFO, 1, 1, error.str().c_str());
        }
        break;
    }

    output->verbose(CALL_INFO, 1, 0, "Callback done\n");
}

void Check::performCallback(SST::Interfaces::StandardMem::WriteResp* resp) {
    SST::SimTime_t curTime = m_tester_ptr->getCurrentSimCycle();
    Addr address = resp->pAddr;

    assert(makeLineAddress(m_address) == makeLineAddress(address));

    output->verbose(CALL_INFO, 1, 0,
        "Callback for address %lx to check in state \"%s\"\n",
        resp->pAddr, printCheckStatus(m_status));

    switch (m_status) {
      case TesterStatus::Action_Pending:
      #if 0
      output->fatal(CALL_INFO, 1, "Should not be here: in WriteResp Check::performCallback 1\n");
      #else
        output->verbose(CALL_INFO, 1, 0,
            "Action callback write value: %d, currently %d\n",
            (m_value + m_store_count), 0);

        m_status = m_store_count == CHECK_SIZE ?
          TesterStatus::Ready : TesterStatus::Idle;
        output->verbose(CALL_INFO, 1, 0,
            "Check %lx, State=%s\n", m_address, printCheckStatus(m_status));

        //m_tester_ptr->decrementOutstandingRequests();
        output->verbose(CALL_INFO, 1, 0, "Action callback return data now %d\n",
                0);
      #endif
        break;

    // Check if the payload is equal to the expected value
      case TesterStatus::Check_Pending: {
        #if 1
        output->fatal(CALL_INFO, 1, "Should not be here: in WriteResp Check::performCallback 2\n");
        #else

          std::stringstream error;
          output->verbose(CALL_INFO, 1, 0, "Check callback\n");
          // Perform load/check

          if (m_tester_ptr->payloadSize() == 1) {
            for (unsigned byte_number = 0; byte_number < CHECK_SIZE; byte_number++) {
                output->verbose(CALL_INFO, 1, 0,
                    "Comparing previously written value with golden reference.\n");
                if (uint8_t(m_value + byte_number) != resp->data[byte_number]) {
                  error << "Action/check failure: proc: " << (resp->id) <<
                    " at address: " << std::hex << address << ", byte_number: " <<
                    byte_number << ", the expected value is " <<
                    static_cast<int>(m_value + byte_number) << " but received " <<
                    static_cast<int>(resp->data[byte_number]) <<
                    " instead. Time: " << std::dec << curTime << "\nTest failed"
                    << std::endl;
                  output->verbose(CALL_INFO, 1, 0, error.str().c_str());
                  output->fatal(CALL_INFO, 1, error.str().c_str());
                }
            }
          }

          output->verbose(CALL_INFO, 1, 0, "Action/check success\n");

          // successful check complete, increment complete
          m_tester_ptr->incrementCheckCompletions();

          m_status = TesterStatus::Idle;
          output->verbose(CALL_INFO, 1, 0, "Check %lx, State=Idle\n", m_address);
          pickValue();
        #endif
        }
        break;

      default: {
        #if 0
          std::stringstream error;
          error << "Unexpected TesterStatus: " << (*this) << " proc: " <<
            resp << " data: ";
          for (auto&& d : resp->data) error << d;
          error << " m_status: " << printCheckStatus(m_status) <<
            " time: " << curTime << "\n";
          output->verbose(CALL_INFO, 1, 1, error.str().c_str());

        #endif
        }
        break;
    }

    output->verbose(CALL_INFO, 1, 0, "Callback done\n");
}

#if 0
// not used anymore
void Check::performCallback(SST::Interfaces::StandardMem::Request* resp) {
    SST::SimTime_t curTime = m_tester_ptr->getCurrentSimCycle();
    Addr address = resp->pAddr;

    assert(makeLineAddress(m_address) == makeLineAddress(address));

    output->verbose(CALL_INFO, 1, 0,
        "Callback for address %lx to check in state \"%s\"\n",
        resp->pAddr, printCheckStatus(m_status));

    switch (m_status) {
      case TesterStatus::Action_Pending:
        output->verbose(CALL_INFO, 1, 0,
            "Action callback write value: %d, currently %d\n",
            (m_value + m_store_count), resp->data[0]);

        m_status = m_store_count == CHECK_SIZE ?
          TesterStatus::Ready : TesterStatus::Idle;
        output->verbose(CALL_INFO, 1, 0,
            "Check %lx, State=%s\n", m_address, printCheckStatus(m_status));

        //m_tester_ptr->decrementOutstandingRequests();
        output->verbose(CALL_INFO, 1, 0, "Action callback return data now %d\n",
                resp->data[0]);
        break;

    // Check if the payload is equal to the expected value
      case TesterStatus::Check_Pending: {
          std::stringstream error;
          output->verbose(CALL_INFO, 1, 0, "Check callback\n");
          // Perform load/check
          if (m_tester_ptr->payloadSize() == 1) {
            for (unsigned byte_number = 0; byte_number < CHECK_SIZE; byte_number++) {
                output->verbose(CALL_INFO, 1, 0,
                    "Comparing previously written value with golden reference.\n");
                if (uint8_t(m_value + byte_number) != resp->data[byte_number]) {
                  error << "Action/check failure: proc: " << (resp->id) <<
                    " at address: " << std::hex << address << ", byte_number: " <<
                    byte_number << ", the expected value is " <<
                    static_cast<int>(m_value + byte_number) << " but received " <<
                    static_cast<int>(resp->data[byte_number]) <<
                    " instead. Time: " << std::dec << curTime << "\nTest failed"
                    << std::endl;
                  output->verbose(CALL_INFO, 1, 0, error.str().c_str());
                  output->fatal(CALL_INFO, 1, error.str().c_str());
                }
            }
          }

          output->verbose(CALL_INFO, 1, 0, "Action/check success\n");

          // successful check complete, increment complete
          m_tester_ptr->incrementCheckCompletions();

          m_status = TesterStatus::Idle;
          output->verbose(CALL_INFO, 1, 0, "Check %lx, State=Idle\n", m_address);
          pickValue();
        }
        break;

      default: {
          std::stringstream error;
          error << "Unexpected TesterStatus: " << (*this) << " proc: " <<
            resp << " data: ";
          for (auto&& d : resp->data) error << d;
          error << " m_status: " << printCheckStatus(m_status) <<
            " time: " << curTime << "\n";
          output->verbose(CALL_INFO, 1, 1, error.str().c_str());
        }
        break;
    }

    output->verbose(CALL_INFO, 1, 0, "Callback done\n");
    delete resp;
}
#endif
void Check::changeAddress(const Addr& address) {
    assert(m_status == TesterStatus::Idle ||
        m_status == TesterStatus::Ready);
    m_status = TesterStatus::Idle;
    m_address = address;
    output->verbose(CALL_INFO, 1, 0, "Check %lx, State=Idle\n", m_address);
    m_store_count = 0;
}

void Check::pickValue() {
    assert(m_status == TesterStatus::Idle);
    m_value = rand() & 0xff;
    m_store_count = 0;
}

void Check::pickInitiatingNode() {
    assert(m_status == TesterStatus::Idle ||
        m_status == TesterStatus::Ready);
    m_status = TesterStatus::Idle;
    m_initiatingNode = rand() % m_num_writers;
    output->verbose(CALL_INFO, 1, 0, "Check %lx, State=Idle, picked initiating node %d\n",
            m_address, m_initiatingNode);
    m_store_count = 0;
}

void Check::print(std::ostream& out) const {
    out << "["
        << m_address << ", value: "
        << static_cast<int>(m_value) << ", status: "
        << printCheckStatus(m_status) << ", initiating node: "
        << m_initiatingNode << ", store_count: "
        << m_store_count
        << "]" << std::flush;
}

void Check::print() const {
  std::stringstream ss;
  ss << "["
    << m_address << ", value: "
    << static_cast<int>(m_value) << ", status: "
    << printCheckStatus(m_status) << ", initiating node: "
    << m_initiatingNode << ", store_count: "
    << m_store_count
    << "]\n";
  output->output(ss.str().c_str());

}


}  // namespace MemHierarchyTester

}  // namespace SST
