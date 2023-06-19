/**
BSD 3-Clause License

This file is part of the RootBA project.
https://github.com/NikolausDemmel/rootba

Copyright (c) 2021-2023, Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#pragma once

namespace rootba {

// number of hardware cores / threads
// This is independent of any restrictions from cgroups / limits.
int hardware_concurrency();

// Maximum concurrecny in the current task arena, not considering any limit set
// by tbb::global_control. Max concurrency is limited by number of hardware
// cores / threads, but also respects any process-wide limits (e.g. cgroups on
// SLURM). The value returned might be lower than the process-wide maximum, if
// executed in a custom task_arena with lower max_concurrecy.
int tbb_task_arena_max_concurrency();

// Current maximum concurrency limit set by tbb::global_control. The effective
// maximum concurrecny in the current task arena might be lower than that.
int tbb_global_max_allowed_parallelism();

// Effective max concurrecny for tasks in the current task arena. Unlike
// tbb_max_concurrency it also considers tbb::global_control and thus should be
// the effective maximum concurrecny of TBB tasks executed in this task arena.
int tbb_effective_max_concurrency();

// set global tbb thread limit; limit is in place until object is destroyed
//
// Usage example:
//
// {
//   ScopedTbbThreadLimit scoped_thread_limit(4);
//
//   tbb::parallel_for(...);
// }
class ScopedTbbThreadLimit {
 public:
  // maximum number of tbb threads (globally); 0 means no limit;
  ScopedTbbThreadLimit(int num_threads);
  ~ScopedTbbThreadLimit();

 private:
  struct Impl;
  Impl* impl_;
};

// NOTE: create before any TBB threads are created!
//
// TODO: test if this works when function using it is run multipl times, when
// the second time around the current thread's task area has already been filled
class TbbConcurrencyObserver {
 public:
  TbbConcurrencyObserver();
  ~TbbConcurrencyObserver();

  int get_current_concurrency() const;
  int get_peak_concurrency() const;

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace rootba
