/*
 * Copyright (c) 2017, Andrea Gaglione, David Rodenas-Herraiz
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \file
 *         fram.c
 * \Contiki port, author 
 *         Rajeev Piyare <rajeev.piyare@hotmail.com>
 * \Adopted from:
 *         Andrea Gaglione <and.gaglione@gmail.com>
 *         David Rodenas-Herraiz <dr424@cam.ac.uk>
 */

#include "contiki.h"
#include "dev/fram.h"
#include <stdio.h>

#ifdef FRAM_START_ADDR_CONF
#define FRAM_START_ADDR FRAM_START_ADDR_CONF
#else
#define FRAM_START_ADDR 0xE000
#endif

#ifdef FRAM_END_ADDR_CONF
#define FRAM_END_ADDR FRAM_END_ADDR_CONF
#else
#define FRAM_END_ADDR 0xFFFF
#endif

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTDEBUG(...)
#endif

/*---------------------------------------------------------------------------*/
uint8_t
fram_write(uint32_t *writeAddress, const void *writeData)
{
  if( ((uintptr_t)writeAddress < FRAM_START_ADDR) || ((uintptr_t)writeAddress > FRAM_END_ADDR) ) {
    PRINTF("%s:: not valid address 0x%" PRIXPTR " - out of bounds\n", __FUNCTION__, (uintptr_t)writeAddress);
    return FRAM_ERROR;
  }

  if( ((uintptr_t)writeAddress >= INTERRUPT_VECTOR_START_ADDR) && ((uintptr_t)writeAddress < INTERRUPT_VECTOR_END_ADDR) ) {
    PRINTF("%s:: not valid address 0x%" PRIXPTR " - memory space allocated for interrupt vectors\n", __FUNCTION__, (uintptr_t)writeAddress);
    return FRAM_ERROR;
  }

  MMIO32(writeAddress) = *(uint32_t*)writeData;
  return FRAM_SUCCESS;
}
/*---------------------------------------------------------------------------*/
uint8_t
fram_read(uint32_t *readAddress, const void *readData)
{
  if( ((uintptr_t)readAddress < FRAM_START_ADDR) || ((uintptr_t)readAddress > FRAM_END_ADDR) ) {
    PRINTF("%s:: not valid address 0x%" PRIXPTR " out of bounds\n", __FUNCTION__, (uintptr_t)readAddress);
    return FRAM_ERROR;
  }

  if( ((uintptr_t)readAddress >= INTERRUPT_VECTOR_START_ADDR) && ((uintptr_t)readAddress < INTERRUPT_VECTOR_END_ADDR) ) {
    PRINTF("%s:: not valid address 0x%" PRIXPTR " memory space allocated for interrupt vectors\n", __FUNCTION__, (uintptr_t)readAddress);
    return FRAM_ERROR;
  }

  *(uint32_t*)readData = MMIO32(readAddress);
  return FRAM_SUCCESS;
}

/*---------------------------------------------------------------------------*/
uint8_t
fram_clear(uint32_t *startAddress, uint32_t *endAddress)
{
  uint32_t dummy = 0xFFFFFFFF;

  if( ((uintptr_t)startAddress < FRAM_START_ADDR) || ((uintptr_t)startAddress > FRAM_END_ADDR) ) {
    PRINTF("%s:: not valid addresses 0x%" PRIXPTR " - 0x%" PRIXPTR "\n", __FUNCTION__, (uintptr_t)startAddress, (uintptr_t)endAddress);
    return FRAM_ERROR;
  }
  if( ((uintptr_t)endAddress < FRAM_START_ADDR) || ((uintptr_t)endAddress > FRAM_END_ADDR) ) {
    PRINTF("%s:: not valid addresses 0x%" PRIXPTR " - 0x%" PRIXPTR "\n", __FUNCTION__, (uintptr_t)startAddress, (uintptr_t)endAddress);
    return FRAM_ERROR;
  }
  while(startAddress < (endAddress - 0x0F)) {
    fram_write(startAddress, &dummy);
    startAddress += sizeof(uint32_t);
  }
  return FRAM_SUCCESS;
}
/*---------------------------------------------------------------------------*/
void
fram_clear_all(void)
{
  uint32_t *startAddress, *endAddress;

  startAddress = (uint32_t *) FRAM_START_ADDR;
  endAddress = (uint32_t *) FRAM_END_ADDR;

  fram_clear(startAddress, endAddress);
}
/*---------------------------------------------------------------------------*/
