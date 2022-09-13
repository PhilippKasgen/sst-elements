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

#ifndef MEM_HIERARCHY_TESTER_CHECKTABLE_HH__
#define MEM_HIERARCHY_TESTER_CHECKTABLE_HH__

#include <iostream>
#include <unordered_map>
#include <vector>

#include "Check.h"
#include "memHierarchyTester.h"


namespace SST {
namespace MemHierarchyTester {

typedef uint64_t Addr;
class MemHierarchyTester;
class Check;

/* as from gem5
 *
*/
template<class T>
static constexpr std::enable_if_t<std::is_integral<T>::value, int>
floorLog2(T x) {
  assert(x > 0);
  uint64_t ux = (typename std::make_unsigned<T>::type) x;

  int y = 0;
  constexpr auto ts = sizeof(T);

  if (ts >= 8 && (ux & 0xffffffff00000000ULL)) { y += 32; ux >>= 32; }
  if (ts >= 4 && (ux & 0x00000000ffff0000ULL)) { y += 16; ux >>= 16; }
  if (ts >= 2 && (ux & 0x000000000000ff00ULL)) { y +=  8; ux >>=  8; }
  if (ux & 0x00000000000000f0ULL) { y += 4; ux >>= 4; }
  if (ux & 0x000000000000000cULL) { y += 2; ux >>= 2; }
  if (ux & 0x0000000000000002ULL) { y += 1; }

  return y;
}

class CheckTable {
 public:
    CheckTable(Output* _output,
        int _num_writers,
        int _num_readers,
        int seed,
        SST::MemHierarchyTester::MemHierarchyTester* _tester);
    ~CheckTable();

    /**
     * Picks a random check from the CheckTable
     */
    Check* getRandomCheck();

    /**
     * Gets the Check handling the specified address
     */
    Check* getCheck(const Addr& address);

    /**
     * Handles the response from memory after a request. If it responds
     * to a flush, read or write, this function will call performCallback
     * of the according Check.
     */
    void handleResponse(Interfaces::StandardMem::Request* ev);


    /* Handler class for StandardMem responses */
    class StdMemHandler : public SST::Interfaces::StandardMem::RequestHandler {
    public:
        friend class CheckTable;
        StdMemHandler(CheckTable* cpuInst, SST::Output* out) : SST::Interfaces::StandardMem::RequestHandler(out), cpu(cpuInst) {}
        virtual ~StdMemHandler() {}
        virtual void handle(SST::Interfaces::StandardMem::ReadResp* rsp) override;
        virtual void handle(SST::Interfaces::StandardMem::WriteResp* rsp) override;
        virtual void handle(SST::Interfaces::StandardMem::FlushResp* rsp) override;
        virtual void handle(SST::Interfaces::StandardMem::InvNotify* rsp) override;

        CheckTable* cpu;
    };


    /**
     * Prints this Check's member variables.
     */
    void print() const;


 private:
    void addCheck(const Addr& address);
    void addCheck(const Addr& address, const unsigned& node_id);

    // Private copy constructor and assignment operator
    CheckTable(const CheckTable& obj);
    CheckTable& operator=(const CheckTable& obj);

    std::vector<Check*> m_check_vector;
    std::unordered_map<Addr, Check*> m_lookup_map;

    Output* output;
    uint64_t readsIssued;
    uint64_t writesIssued;
    uint64_t flushsIssued;
    const int m_num_writers;
    const int m_num_readers;
    SST::MemHierarchyTester::MemHierarchyTester* m_tester_ptr;
    StdMemHandler* stdMemHandlers;
};
}  // namespace MemHierarchyTester
}  // namespace SST

#endif  // MEM_HIERARCHY_TESTER_CHECKTABLE_HH__
