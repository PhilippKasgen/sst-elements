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

#ifndef MEM_HIERARCHY_TESTER_CHECK_H
#define MEM_HIERARCHY_TESTER_CHECK_H

#include <iostream>
#include "memHierarchyTester.h"


namespace SST {

namespace MemHierarchyTester {

class Check;
class MemHierarchyTester;
typedef uint64_t Addr;

typedef enum { Idle, Action_Pending, Ready, Check_Pending } TesterStatus;

const char* printCheckStatus(const TesterStatus& s);

const int CHECK_SIZE_BITS = 2;
const int CHECK_SIZE = (1 << CHECK_SIZE_BITS);

class Check {
 public:
    Check(Output* output, Addr address, int _num_writers, int _num_readers,
        SST::MemHierarchyTester::MemHierarchyTester* _tester);

    Check(Output* output, Addr address, int _num_writers, int _num_readers,
      unsigned node_index_to_bind,
      SST::MemHierarchyTester::MemHierarchyTester* _tester);

    ~Check() {};

    /**
     * This function is called by the MemHierarchyTester. If flushes are enabled,
     * this will randomly perform a flush and wait for the memory hierarchy to
     * respond to this request. Otherwise this will proceed with initiate2().
     */
    void initiate();

    /**
     * This function performs either an action (initiateAction()), or a check
     * (initiateCheck()), depending on the Check's state.
     */
    void initiate2();

    /**
     * When the memory responds to a request, this function will be called.
     * If this Check is in Action_Pending state, the Check will automatically
     * return to Idle state. If this Check is in Check_Pending state, it will
     * check whether the values previously written by this Check to memory
     * can be correctly read back. If this fails, the simulation stops.
     */
    void performCallback(SST::Interfaces::StandardMem::ReadResp* resp);
    void performCallback(SST::Interfaces::StandardMem::WriteResp* resp);

    /**
     * returns the address member
     */
    Addr getAddress() const { return m_address; }

    /**
     * Sets the address member
     */
    void changeAddress(const Addr& address);

    /**
     * Prints this Check's member variables to output
     */
    void print() const;

    /**
     * Prints this Check's member variables to the out ostream object
     */
    void print(std::ostream& out) const;

    inline static const std::string debug_file_name_prefix = "MemHierarchyTester_debug_";
    inline static const std::string debug_file_name_suffix = ".trace";
 private:
    void initiateFlush();
    void initiatePrefetch();
    void initiateAction();
    void initiateCheck();
    Addr makeLineAddress(const Addr& address) const;

    void pickValue();
    void pickInitiatingNode();

    Output* output;
    TesterStatus m_status;
    uint8_t m_value;
    unsigned m_store_count;
    unsigned m_initiatingNode;
    Addr m_address;
    const int m_num_writers;
    const int m_num_readers;
    MemHierarchyTester* m_tester_ptr;
};

inline std::ostream&
operator<<(std::ostream& out, const Check& obj) {
    obj.print(out);
    out << std::flush;
    return out;
}

}  // namespace MemHierarchyTester

}  // namespace SST

#endif  // MEM_HIERARCHY_TESTER_CHECK_H
