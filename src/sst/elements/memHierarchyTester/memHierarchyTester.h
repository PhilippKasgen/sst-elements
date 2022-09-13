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

#ifndef MEM_HIERARCHY_TESTER_H
#define MEM_HIERARCHY_TESTER_H


#include <vector>

#include "sst/core/output.h"
#include "sst/core/component.h"
#include "sst/core/params.h"
#include "sst/core/event.h"
#include "sst/core/sst_types.h"
#include "sst/core/link.h"
#include "sst/core/interfaces/stdMem.h"

#include "CheckTable.h"
#include "Check.h"

#ifdef HAVE_LIBZ
#include <zlib.h>
#endif

namespace SST {

namespace MemHierarchyTester {

class Check;
class CheckTable;
typedef uint32_t flags_t;

class MemHierarchyTester : public SST::Component {
 public:
  MemHierarchyTester(ComponentId_t id, const Params& params);
  ~MemHierarchyTester();

  void setup() { }
  void init(unsigned int phase);
  void finish();

  SST_ELI_REGISTER_COMPONENT(
        MemHierarchyTester,
        "memHierarchyTester",
        "randomTester",
        SST_ELI_ELEMENT_VERSION(1, 0, 0),
        "MemHierarchyTester CPU Memory Hierarchy Tester Engine",
        COMPONENT_CATEGORY_PROCESSOR
  )

  SST_ELI_DOCUMENT_PARAMS(
  { "verbose", "Verbosity for debugging. Increased numbers for increased verbosity.", "0" },
  { "clock", "Sets the clock of the core", "2GHz"} ,
  { "checks", "How many checks should be performed", "100"},
  { "num_cores", "Number of cores", ""},
  { "deadlock_threshold", "Timeout limit to detect deadlocks", "100"},
  { "random_seed", "Random seed for the random check generation", "2"},
  { "check_flush", "Whether flushes are to test along with the usual checks", "false"},
  { "payload_size", "Adjust the payload size in bytes for reading and writing. Values other than \
1 disable the value check and just generate traffic.", "1"},
  { "timeout", "Maximum time a simulation might run", ""},
  { "cache_line_size", "Sets the length of the cache line in bytes, this should match the L1 cache", "64" },
  { "tie_mem_addresses", "If enabled, the address space is partitioned for all cores. If disabled, \
    request addresses are randomly distributed over all cores", "0" }
  )

  SST_ELI_DOCUMENT_PORTS(
  { "cache_link_%(num_cores)d",
    "Each core's link to its cache", { "memHierarchy.memEvent", "" } }
  )

  SST_ELI_DOCUMENT_SUBCOMPONENT_SLOTS(
  {"memory", "Interface to the memory hierarchy (e.g., cache)", "SST::Interfaces::StandardMem" }
  )

  /**
   * Gets a CPU port which is connected to a L1D cache
   */
  Interfaces::StandardMem* getWritableCpuPort(const unsigned& idx);

  /*
   * Gets a CPU port which is either a L1D or L1I cache
   */
  Interfaces::StandardMem* getReadableCpuPort(const unsigned& idx);

  /*
   * Return whether the indexed CPU port is only for instructions
   */
  bool isInstOnlyCpuPort(const unsigned& idx) const;

  /*
   * Returns whether the indexed CPU Port is connected to an instruction or data cache
   */
  bool isInstDataCpuPort(const unsigned& idx) const;

  /*
   * Increments the completed checks
   */
  void incrementCheckCompletions();

  /*
   * Returns whether flushes should be performed
   */
  bool getCheckFlush() const;

  /*
   * Increments the number of checks in preparation which means they are not ready to initiate the check yet.
   */
  void incrementChecksInPreparation();

  /*
   * Increments the checks which can be checked, and decrements the checks in preparation
   */
  void incrementChecksReady();

  /*
   * Updates the last progress time and indicates that a response is expected from the memory hierarchy
   */
  void madeProgress(const unsigned& cpuIndex);

  /*
   * resets the progress time such that it indicates that no response is expected from memory at this point
   * and therefore prohibits timing out
   */
  void resetProgress(const unsigned& cpuIndex);

  /*
   * returns the maximal payload size. If this is set to 1, previously written values will be compared
   * to an expected value. If it is set to another value than 1, the random tester turns into a
   * random traffic generator.
   */
  uint32_t payloadSize() const { return payload_size; }

  /*
   * Return the cache line size
   */
  uint32_t cachelineSize() const { return cache_line_size; }

  bool areAddressesTied() const { return m_tie_addresses; }

 private:
  MemHierarchyTester();                         // Serialization only
  MemHierarchyTester(const MemHierarchyTester&); // Do not impl.
  void operator=(const MemHierarchyTester&);    // Do not impl.

  bool tick(Cycle_t);

  Output* output;
  uint64_t pageSize;
  uint32_t maxOutstanding;
  uint32_t maxIssuePerCycle;

  uint64_t readsIssued;
  uint64_t writesIssued;
  uint64_t splitReadsIssued;
  uint64_t splitWritesIssued;
  uint64_t totalBytesRead;
  uint64_t totalBytesWritten;
  uint64_t cyclesWithIssue;
  uint64_t cyclesWithNoIssue;

  void checkForDeadlock(const SST::Cycle_t& current_time) const;

  std::vector<SST::Interfaces::StandardMem*> cache_links;
  SST::MemHierarchyTester::CheckTable* m_checkTable_ptr;

  std::vector<SST::Cycle_t> m_last_progress_vector;
  uint64_t m_checks_completed;
  uint64_t m_checksInPreparation;
  uint64_t m_checksReady;
  const uint64_t m_checks_to_complete;
  const bool m_check_flush;
  const Cycle_t m_deadlock_threshold;
  const int m_num_cpus;
  const Cycle_t m_timeout;
  const bool m_tie_addresses;
  const uint32_t payload_size;
  const uint32_t cache_line_size;
};

}  // namespace MemHierarchyTester
}  // namespace SST
#endif /* MEM_HIERARCHY_TESTER_H */
