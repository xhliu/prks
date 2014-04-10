/* $Id: PoolP.nc,v 1.7 2010-01-20 19:59:07 scipio Exp $ */
/*
 * Copyright (c) 2006 Stanford University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Stanford University nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL STANFORD
 * UNIVERSITY OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/**
 *  Implementation of a general dynamic memory pool component.
 *  Note that the allocation/deallocation policies are 
 *  different than traditional dynamic allocators such as
 *  malloc or slab allocators. When initialized, the Pool
 *  contains <code>size</code> items of type <code>pool_t</code>.
 *  These elements can be removed from the pool for use with 
 *  <code>Pool.get</code>, and new elements can be placed in
 *  the pool with <code>Pool.put</code>. The pool allows 
 *  components to <code>put</code> elements besides those which
 *  were obtained with <code>get</code>. The pool can never have
 *  more than <code>size</code> elements in it.
 *
 *  @author Philip Levis
 *  @author Kyle Jamieson
 *  @author Geoffrey Mainland
 *  @date   $Date: 2010-01-20 19:59:07 $
 */

generic module AsyncPoolP(typedef pool_t, uint8_t size) {
  provides {
    interface Init;
    interface AsyncPool<pool_t> as Pool;
  }
}
implementation {
  uint8_t free;
  uint8_t index;
  pool_t* ONE_NOK queue[size];
  pool_t pool[size];

  command error_t Init.init() {
    int i;
    for (i = 0; i < size; i++) {
      queue[i] = &pool[i];
    }
    free = size;
    index = 0;
    return SUCCESS;
  }
  
  async command bool Pool.empty() {
    dbg("PoolP", "%s size is %i\n", __FUNCTION__, (int)free);
    atomic return free == 0;
  }
  async command uint8_t Pool.size() {
    dbg("PoolP", "%s size is %i\n", __FUNCTION__, (int)free);
    atomic return free;
  }
    
  async command uint8_t Pool.maxSize() {
    atomic return size;
  }

  async command pool_t* Pool.get() {
atomic {
    if (free) {
      pool_t* rval = queue[index];
      queue[index] = NULL;
      free--;
      index++;
      if (index == size) {
        index = 0;
      }
      dbg("PoolP", "%s size is %i\n", __FUNCTION__, (int)free);
      return rval;
    }
}
    return NULL;
  }

  async command error_t Pool.put(pool_t* newVal) {
atomic {
    if (free >= size) {
      return FAIL;
    }
    else {
      uint16_t emptyIndex = (index + free);
      if (emptyIndex >= size) {
        emptyIndex -= size;
      }
      queue[emptyIndex] = newVal;
      free++;
      dbg("PoolP", "%s size is %i\n", __FUNCTION__, (int)free);
      return SUCCESS;
    }
  }
}

}
