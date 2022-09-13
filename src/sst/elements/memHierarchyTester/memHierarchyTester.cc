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

#include "sst/core/sst_config.h"
#include <sst/core/unitAlgebra.h>

#include "memHierarchyTester.h"
#include <algorithm>
#include <string>
#include <fstream>

namespace SST {
namespace MemHierarchyTester {


  MemHierarchyTester::MemHierarchyTester(ComponentId_t id, const Params& params) :
  Component(id),
  m_checks_completed(0),
  m_checks_to_complete(static_cast<uint64_t>(
        params.find<uint64_t>("checks"))),
  m_check_flush(static_cast<bool>(params.find<bool>("check_flush", false))),
  m_deadlock_threshold(static_cast<Cycle_t>(
        params.find<Cycle_t>("deadlock_threshold"))),
  m_num_cpus(static_cast<int>(params.find<int>("num_cores", 1))),
  m_timeout(static_cast<Cycle_t>(params.find<Cycle_t>("timeout"))),
  m_tie_addresses(static_cast<bool>(params.find<uint32_t>("tie_mem_addresses", 0)) == 1),
  payload_size(static_cast<uint32_t>(params.find<uint32_t>("payload_size", 1))),
  cache_line_size(static_cast<uint32_t>(params.find<uint32_t>("cache_line_size", 64)))
  {
  assert(payload_size > 0);
  const uint32_t output_level = static_cast<uint32_t>(params.find<uint32_t>("verbose", 4));
  output = new SST::Output("MemHierarchyTester[@p:@l]: ",
      output_level, 0, SST::Output::STDOUT);

  output->verbose(CALL_INFO, 1, 0, "number of cores: %d\n", m_num_cpus);
  output->verbose(CALL_INFO, 1, 0, "timeout set to %ld\n", m_timeout);
  output->verbose(CALL_INFO, 1, 0, "checks to complete: %ld\n", m_checks_to_complete);
  output->verbose(CALL_INFO, 1, 0, "timeout for deadlock detection: %ld\n", m_deadlock_threshold);
  std::string prosClock = params.find<std::string>("clock", "2GHz");
  // Register the clock
  TimeConverter* time = registerClock(prosClock,
      new Clock::Handler<MemHierarchyTester>(this,
        &MemHierarchyTester::tick));

  m_checksInPreparation = 0;
  m_checksReady = 0;

  if (output_level > 1) {
    for (int i = 0; i < m_num_cpus; i++) {
      std::string f_name = Check::debug_file_name_prefix + std::to_string(i) + Check::debug_file_name_suffix;
      std::ofstream tmp(f_name);
      tmp << ""  << std::flush;
      tmp.close();
    }
  }

  output->verbose(CALL_INFO, 1, 0,
      "Configured MemHierarchyTester clock for %s\n", prosClock.c_str());

  int seed = static_cast<int>(
      params.find<int>("random_seed", 1));
  m_checkTable_ptr = new SST::MemHierarchyTester::CheckTable(
        output,
        m_num_cpus,  // m_num_writers,
        m_num_cpus,  // m_num_readers,
        seed,
        this);

  output->verbose(CALL_INFO, 1, 0,
      "Configuring MemHierarchyTester cache connection...\n");
  auto handler = new SST::Interfaces::StandardMem::Handler<CheckTable>(
      m_checkTable_ptr,
      &CheckTable::handleResponse);
  int idx = 0;

  for (int i = 0; i < m_num_cpus; ++i) {
    Params par;
    std::string temp = "cache_link_"+std::to_string(idx);
    par.insert("port", temp);
    auto port = loadAnonymousSubComponent<Interfaces::StandardMem>(
        "memHierarchy.standardInterface", "memory", i,
        ComponentInfo::INSERT_STATS | ComponentInfo::SHARE_PORTS, par, time,
        handler);
    cache_links.push_back(port);
    idx++;
  }

  output->verbose(CALL_INFO, 1, 0,
      "Configuration of memory interface completed.\n");

  // noone's left behind
  registerAsPrimaryComponent();
  primaryComponentDoNotEndSim();

  output->verbose(CALL_INFO, 1, 0,
      "MemHierarchyTester configuration completed successfully.\n");
}

MemHierarchyTester::~MemHierarchyTester() {
  // delete memMgr;
  delete output;

  delete m_checkTable_ptr;
  for (unsigned i = 0; i < cache_links.size(); i++)
    delete cache_links[i];
}

void MemHierarchyTester::init(unsigned int phase) {
  for (auto&& cache_link : cache_links) {
    cache_link->init(phase);
  }
  Cycle_t tmp = static_cast<Cycle_t>(0xffffffffffffffff);
  for (int i = 0; i < m_num_cpus; i++) {
    m_last_progress_vector.push_back(tmp);
  }
}


void MemHierarchyTester::finish() {


  const uint64_t nanoSeconds = getCurrentSimTimeNano();

  output->output("\n");
  output->output("MemHierarchyTester Component Statistics:\n");

  output->output("------------------------------------------------------------------------\n");
  output->output("- Completed at:                          %" PRIu64 " ns\n", nanoSeconds);
  output->output("- Cycles with ops issued:                %" PRIu64 " cycles\n", cyclesWithIssue);
  output->output("------------------------------------------------------------------------\n");
  output->output("- Cycles with no ops issued (LS full):   %" PRIu64 " cycles\n", cyclesWithNoIssue);
  m_checkTable_ptr->print();
  output->output("------------------------------------------------------------------------\n");
  output->output("Tests completed: %" PRIu64 " of %" PRIu64 "\n", m_checks_completed, m_checks_to_complete);
  output->output("Tests in preparation: %" PRIu64 ", Tests ready: %" PRIu64 "\n", m_checksInPreparation,
      m_checksReady);
  output->output("Test successful\n");
}

void MemHierarchyTester::madeProgress(const unsigned& cpuIndex) {
  this->m_last_progress_vector[cpuIndex] = getCurrentSimTimeNano();
}

void MemHierarchyTester::resetProgress(const unsigned& cpuIndex) {
  this->m_last_progress_vector[cpuIndex] = static_cast<Cycle_t>(0xffffffffffffffff);
}

void MemHierarchyTester::incrementChecksInPreparation() {
  this->m_checksInPreparation++;
}

void MemHierarchyTester::incrementChecksReady() {
  this->m_checksInPreparation--;
  this->m_checksReady++;
}

bool MemHierarchyTester::tick(SST::Cycle_t currentCycle) {

  // if there are more checks to do...
  if (this->m_checks_completed < this->m_checks_to_complete) {
    // get a random check
    Check* check_ptr = this->m_checkTable_ptr->getRandomCheck();
    // assert that we got a valid Check
    assert(check_ptr != nullptr);

    // issue new request
    check_ptr->initiate();

    // check for deadlocks
    this->checkForDeadlock(currentCycle);
  } else {
    primaryComponentOKToEndSim();
    return true;
  }
  if (currentCycle > m_timeout) {
    std::stringstream error;
    error << "Simulation timed out.\n currentCycle: " << currentCycle << " Tests in preparation: " <<
      this->m_checksInPreparation << "\n  Tests ready: " <<
      this->m_checksReady << "\n  Tests completed: " <<
      this->m_checks_completed << "\nTest failed" << std::endl;
    output->output(error.str().c_str());
    this->m_checkTable_ptr->print();
    output->fatal(CALL_INFO, 1, error.str().c_str());
  }
  // Keep simulation ticking, we have more work to do if we reach here
  return false;
}

void MemHierarchyTester::checkForDeadlock(
    const SST::Cycle_t& current_time) const {
    unsigned size = (this->m_last_progress_vector).size();
    for (unsigned processor = 0; processor < size; processor++) {
        SST::Cycle_t last_progress = (this->m_last_progress_vector)[processor];
        if (current_time > last_progress && current_time - last_progress > static_cast<unsigned>(this->m_deadlock_threshold)) {
          // the following message was printed with gem5's #panic()
          std::stringstream error;
          error << "Deadlock detected: current_time: " << current_time <<
            " last_progress_time: " << last_progress <<
            " difference: " << (current_time - last_progress) <<
            " processor: " << processor <<
            "\nTests in preparation: " << this->m_checksInPreparation <<
            "  Tests ready: " << this->m_checksReady <<
            "  Tests completed: " << this->m_checks_completed <<
            "  Tests to complete: " << this->m_checks_to_complete <<
            "\nTest failed" << std::endl;
          output->verbose(CALL_INFO, 1, 1, error.str().c_str());
          output->fatal(CALL_INFO, 1, error.str().c_str());
        }
    }
}

SST::Interfaces::StandardMem* MemHierarchyTester::getWritableCpuPort(const unsigned& idx) {
  assert(idx >= 0 && idx < static_cast<unsigned>((this->cache_links).size()));
  return (this->cache_links[idx]);
}

SST::Interfaces::StandardMem* MemHierarchyTester::getReadableCpuPort(const unsigned& idx) {
  assert(idx >= 0 && idx < static_cast<unsigned>((this->cache_links).size()));
  return (this->cache_links[idx]);
}

bool MemHierarchyTester::isInstDataCpuPort(const unsigned& idx) const {
  return true;
}

bool MemHierarchyTester::isInstOnlyCpuPort(const unsigned& idx) const {
  return false;
}

void MemHierarchyTester::incrementCheckCompletions() {
  (this->m_checks_completed)++;
  (this->m_checksReady)--;
  output->verbose(CALL_INFO, 1, 0, "Tests completed %ld\n", (this->m_checks_completed));
}

bool MemHierarchyTester::getCheckFlush() const {
  return this->m_check_flush;
}
}  // namespace MemHierarchyTester
}  // namespace SST
