/*
 * Copyright (C) 2015 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file arch/chibios/subsystems/settings_arch.c
 * Persistent settings low level flash routines stm32.
 *
 * file flash_functions.c
 *
 */

#include <string.h>
#include "std.h"
#include "hal.h"
#include "flash_functions.h"

//struct FLASH_TypeDef FLASH;


//bool move_flash_boot_address(uint16_t flash_boot_address);
static int flash_Write_32bit(flashaddr_t address, uint32_t* buffer, uint32_t size);
static int flash_Write_8bit(flashaddr_t address, const char* buffer, size_t size);
static void flashWriteData(flashaddr_t address, const flashdata_t data);
#if defined(STM32F7) 
static bool flash_Unlock(void);
static void flash_lock(void);
static void flash_WaitWhileBusy(void);
#elif defined(STM32H7)
static bool flash_Unlock(uint8_t bank);
static void flash_lock(uint8_t bank);
static void flash_WaitWhileBusy(uint8_t bank);
#endif

bool write_to_flash_32bit(uint8_t sector_nb, uint32_t* buffer_ptr, uint32_t size)
{
//flashsector_t sector;

  // 88 = sector4 on ICTM interfase (0x00220000) and 2004 = sector4 in AXIM interface (0x08020000).
  // Whatever hex address you need to move the origin to just shft the address 14 bits to the right.
  // move_flash_boot_address(88); 
  
  if (flashSectorErase(sector_nb)) { return -1; }

  flash_Write_32bit(flashSectorBegin(sector_nb), buffer_ptr, size);


return 0;
}

bool write_to_flash_8bit(uint8_t sector_nb, const char* buffer_ptr, uint32_t size)
{
//flashsector_t sector;

  // 88 = sector4 on ICTM interfase (0x00220000) and 2004 = sector4 in AXIM interface (0x08020000).
  // Whatever hex address you need to move the origin to just shft the address 14 bits to the right.
  // move_flash_boot_address(88); 
  
  if (flashSectorErase(sector_nb)) { return -1; }

  flash_Write_8bit(flashSectorBegin(sector_nb), buffer_ptr, size);


return 0;
}



size_t flashSectorSize(flashsector_t sector)
{

#if defined(STM32F4)
    if (sector <= 3)
        return(16 * 1024);
    else if (sector == 4)
        return(64 * 1024);
    else if (sector >= 5 && sector <= 11)
        return(128 * 1024);
    return 0;
#elif defined(STM32F7)
// sectors 0..11 are the 1st memory bank (1Mb), and 12..23 are the 2nd (the same structure).
  if (sector <= 3 || (sector >= 12 && sector <= 15))
    return(32 * 1024);
  else if (sector == 4 || sector == 16)
    return(128 * 1024);
  else if ((sector >= 5 && sector <= 11) || (sector >= 17 && sector <= 23))
    return(256 * 1024);
#elif defined(STM32H7)
// sectors 0..11 are the 1st memory bank (1Mb), and 12..23 are the 2nd (the same structure).
  if (sector <= 7)
    return(128 * 1024);
  else if (sector >= 8 && sector <= 15)
    return(128 * 1024);
#endif

//return (128 * 1024);
return 0;
}



flashaddr_t flashSectorBegin(flashsector_t sector)
{
    flashaddr_t address = FLASH_BASE;
    while (sector > 0)
    {
        --sector;
        address += flashSectorSize(sector);
    }
    return address;
}

flashaddr_t flashSectorEnd(flashsector_t sector)
{
    return flashSectorBegin(sector + 1);
}

flashsector_t flashSectorAt(flashaddr_t address)
{
    flashsector_t sector = 0;
    while (address >= flashSectorEnd(sector))
        ++sector;
    return sector;
}


#if defined(STM32F7) 

static void flash_WaitWhileBusy(void)
{
  while(FLASH->SR & FLASH_SR_BSY);

return;
}

#elif defined(STM32H7)

static void flash_WaitWhileBusy(uint8_t bank)
{
   if (bank == 1) {
     while(FLASH->SR1 & FLASH_SR_BSY);
   } else if (bank == 2) {
     while(FLASH->SR2 & FLASH_SR_BSY);
   } 

return;
}

#endif




#if defined(STM32F7) 

static void flash_lock(void)
{
  FLASH->CR |= FLASH_CR_LOCK;

return;
}

#elif defined(STM32H7)

static void flash_lock(uint8_t bank)
{
   if (bank == 1) {
     FLASH->CR1 |= FLASH_CR_LOCK;
   } else if (bank == 2) {
     FLASH->CR2 |= FLASH_CR_LOCK;
   } 

return;
}

#endif


/**
 * @brief Wait for the flash operation to finish.
 */

/**
 * @brief Unlock the flash memory for write access.
 * @return CH_SUCCESS  Unlock was successful.
 * @return CH_FAILED    Unlock failed.
 */
#if defined(STM32F7) 
static bool flash_Unlock(void)
{
   /* Check if unlock is really needed */
    if (!(FLASH->CR & FLASH_CR_LOCK))
        return CH_SUCCESS;

    /* Write magic unlock sequence */
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;

    /* Check if unlock was successful */
    if (FLASH->CR & FLASH_CR_LOCK)
        return CH_FAILED;

return CH_SUCCESS;
}
#elif defined(STM32H7)
static bool flash_Unlock(uint8_t bank)
{

  if (bank == 1) {
    /* Check if unlock is really needed */
    if (!(FLASH->CR1 & FLASH_CR_LOCK)) {
        return CH_SUCCESS;
    }
    /* Write magic unlock sequence */
    FLASH->KEYR1 = 0x45670123;
    FLASH->KEYR1 = 0xCDEF89AB;

    /* Check if unlock was successful */
    if (FLASH->CR1 & FLASH_CR_LOCK) {
        return CH_FAILED;
    }
  } else if (bank == 2) {
    /* Check if unlock is really needed */
    if (!(FLASH->CR2 & FLASH_CR_LOCK)) {
        return CH_SUCCESS;
    }
    /* Write magic unlock sequence */
    FLASH->KEYR2 = 0x45670123;
    FLASH->KEYR2 = 0xCDEF89AB;

    /* Check if unlock was successful */
    if (FLASH->CR2 & FLASH_CR_LOCK) {
        return CH_FAILED;
    }
  }

return CH_SUCCESS;
}
#endif



/**
 * @brief Lock the flash memory for write access.
 */

int flashSectorErase(flashsector_t sector)
{
    chSysLock();
#if defined(STM32F7)
    /* Unlock flash for write access */
    if(flash_Unlock() == CH_FAILED){ return FLASH_RETURN_NO_PERMISSION; }
    /* Wait for any busy flags. */
    flash_WaitWhileBusy();
    /* Setup parallelism before any program/erase */
    FLASH->CR &= ~FLASH_CR_PSIZE_MASK;
    FLASH->CR |= FLASH_CR_PSIZE_VALUE;
    // SINGLE BANK MODE sectors=11, 00000 = sector 0, 00001 = sector 1, ..., 01011 = sector 11, Others not allowed.
    // DUAL BANK MODE sectors=23, 10000 = sector 12, 10001 = sector 13, ..., 11011 = sector 23, Others not allowed.
    // only the FLASH_CR_SNB_4 bit changes so in effect we have 2 banks of 11 sectors.
    FLASH->CR &= ~(FLASH_CR_SNB_0 | FLASH_CR_SNB_1 | FLASH_CR_SNB_2 | FLASH_CR_SNB_3 | FLASH_CR_SNB_4);
    if (sector <= 11) {
     FLASH->CR |= (sector << FLASH_CR_SNB_Pos);
    } else if (sector > 11 && sector <= 23) {
      FLASH->CR |= ((sector-12) << FLASH_CR_SNB_Pos);
      FLASH->CR |= FLASH_CR_SNB_4; 
    } else {
      return FLASH_RETURN_BAD_FLASH;
    }

    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;

    /* Wait until it's finished. */
    flash_WaitWhileBusy();

    /* Sector erase flag does not clear automatically. */
    FLASH->CR &= ~FLASH_CR_SER;
    /* Lock flash again */
    flash_lock();
    chSysUnlock();
    /* Check deleted sector for errors */
    if (flashIsErased(flashSectorBegin(sector), flashSectorSize(sector)) == FALSE){
      return FLASH_RETURN_BAD_FLASH;  /* Sector is not empty despite the erase cycle! */
    }

#elif defined(STM32H7)
    if(sector <= 7) {

      /* Unlock flash for write access */
      if(flash_Unlock(1) == CH_FAILED){ return FLASH_RETURN_NO_PERMISSION; }
      /* Wait for any busy flags. */
      flash_WaitWhileBusy(1);
      /* Setup parallelism before any program/erase */
      FLASH->CR1 &= ~(FLASH_CR_SNB_0 | FLASH_CR_SNB_1 | FLASH_CR_SNB_2 );
      FLASH->CR1 |= (sector << (FLASH_CR_SNB_Pos));
      FLASH->CR1 |= FLASH_CR_SER;
      FLASH->CR1 |= FLASH_CR_START;
      /* Wait until it's finished. */
      flash_WaitWhileBusy(1);
      /* Sector erase flag does not clear automatically. */
      FLASH->CR1 &= ~FLASH_CR_SER;
      /* Lock flash again */
      flash_lock(1);
      /* Check deleted sector for errors */
      if (flashIsErased(flashSectorBegin(sector), flashSectorSize(sector)) == FALSE){
        return FLASH_RETURN_BAD_FLASH;  /* Sector is not empty despite the erase cycle! */
      }

    } else if (sector >= 8 && sector <= 15) {
      /* Unlock flash for write access */
      if(flash_Unlock(2) == CH_FAILED){ return FLASH_RETURN_NO_PERMISSION; }
      /* Wait for any busy flags. */
      flash_WaitWhileBusy(2);
      /* Setup parallelism before any program/erase */
      FLASH->CR2 &= ~(FLASH_CR_SNB_0 | FLASH_CR_SNB_1 | FLASH_CR_SNB_2 );
      FLASH->CR2 |= (sector << (FLASH_CR_SNB_Pos));
      FLASH->CR2 |= FLASH_CR_SER;
      FLASH->CR2 |= FLASH_CR_START;
      /* Wait until it's finished. */
      flash_WaitWhileBusy(2);
      /* Sector erase flag does not clear automatically. */
      FLASH->CR2 &= ~FLASH_CR_SER;
      /* Lock flash again */
      flash_lock(2);
      chSysUnlock();
      /* Check deleted sector for errors */
      if (flashIsErased(flashSectorBegin(sector), flashSectorSize(sector)) == FALSE){
        return FLASH_RETURN_BAD_FLASH;  /* Sector is not empty despite the erase cycle! */
      }
    } else {
      return FLASH_RETURN_BAD_FLASH;
    }

#endif

      chSysUnlock();

/* Successfully deleted sector */
return FLASH_RETURN_SUCCESS;
}

int flash_Erase(flashaddr_t address, size_t size)
{
    chSysLock();
    while (size > 0)
    {
        flashsector_t sector = flashSectorAt(address);
        int err = flashSectorErase(sector);
        if (err != FLASH_RETURN_SUCCESS) {
          chSysUnlock(); 
          return err;
        }
        address = flashSectorEnd(sector);
        size_t sector_size = flashSectorSize(sector);
        if (sector_size >= size)
            break;
        else
            size -= sector_size;
    }
    chSysUnlock();  //chSysUnlockFromISR();  
    return FLASH_RETURN_SUCCESS;
}

bool flashIsErased(flashaddr_t address, size_t size)
{
    /* Check for default set bits in the flash memory
     * For efficiency, compare flashdata_t values as much as possible,
     * then, fallback to byte per byte comparison. */
    while (size >= sizeof(flashdata_t))
    {
        if (*(volatile flashdata_t*)address != (flashdata_t)(-1)){ // flashdata_t being unsigned, -1 is 0xFF..FF
            return false;
        }
        address += sizeof(flashdata_t);
        size -= sizeof(flashdata_t);
    }
    while (size > 0)
    {
        if (*(char*)address != 0xff)
            return false;
        ++address;
        --size;
    }

return true;
}

bool flash_Compare(flashaddr_t address, const char* buffer, size_t size)
{
    /* For efficiency, compare flashdata_t values as much as possible,
     * then, fallback to byte per byte comparison. */
    while (size >= sizeof(flashdata_t))
    {
        if (*(volatile flashdata_t*)address != *(flashdata_t*)buffer)
            return false;
        address += sizeof(flashdata_t);
        buffer += sizeof(flashdata_t);
        size -= sizeof(flashdata_t);
    }
    while (size > 0)
    {
        if (*(volatile char*)address != *buffer)
            return false;
        ++address;
        ++buffer;
        --size;
    }

return true;
}

int flash_Read(flashaddr_t address, char* buffer, size_t size)
{
    memcpy(buffer, (char*)address, size);
    return FLASH_RETURN_SUCCESS;
}

static void flashWriteData(flashaddr_t address, const flashdata_t data)
{
#if defined(STM32F7)
    /* Enter flash programming mode */
    FLASH->CR |= FLASH_CR_PG;
    /* Write the data */
    *(flashdata_t*)address = data;
    /* Wait for completion */
    flash_WaitWhileBusy();
    /* Exit flash programming mode */
    FLASH->CR &= ~FLASH_CR_PG;
#elif defined(STM32H7)
  if (address <= 0x080FFFFF && FLASH_END >= 0x081FFFFF) { // 2MB devices
    /* Enter flash programming mode */
    FLASH->CR1 |= FLASH_CR_PG;
    /* Write the data */
    *(flashdata_t*)address = data;
    /* Wait for completion */
    flash_WaitWhileBusy(1);
    /* Exit flash programming mode */
    FLASH->CR1 &= ~FLASH_CR_PG;
  } else if (address > 0x080FFFFF && FLASH_END >= 0x081FFFFF) {
    /* Enter flash programming mode */
    FLASH->CR2 |= FLASH_CR_PG;
    /* Write the data */
    *(flashdata_t*)address = data;
    /* Wait for completion */
    flash_WaitWhileBusy(2);
    /* Exit flash programming mode */
    FLASH->CR2 &= ~FLASH_CR_PG;
  } else if (address <= 0x0807FFFF && FLASH_END <= 0x0817FFFF) { // 1 MB devices.
    /* Enter flash programming mode */
    FLASH->CR1 |= FLASH_CR_PG;
    /* Write the data */
    *(flashdata_t*)address = data;
    /* Wait for completion */
    flash_WaitWhileBusy(1);
    /* Exit flash programming mode */
    FLASH->CR1 &= ~FLASH_CR_PG;
  } else if (FLASH_END > 0x0807FFFF && FLASH_END <= 0x0817FFFF) {
    /* Enter flash programming mode */
    FLASH->CR2 |= FLASH_CR_PG;
    /* Write the data */
    *(flashdata_t*)address = data;
    /* Wait for completion */
    flash_WaitWhileBusy(2);
    /* Exit flash programming mode */
    FLASH->CR2 &= ~FLASH_CR_PG;
  }
#endif
}


int flash_Write_32bit(flashaddr_t address, uint32_t* buffer, uint32_t size)
{
    chSysLock();

#if defined(STM32F7)
    /* Unlock flash for write access */
    if(flash_Unlock() == CH_FAILED)
      return FLASH_RETURN_NO_PERMISSION;
    /* Wait for any busy flags */
    flash_WaitWhileBusy();
    /* Setup parallelism before any program/erase */
    FLASH->CR &= ~FLASH_CR_PSIZE_MASK;
    FLASH->CR |= FLASH_CR_PSIZE_VALUE;
    while (size >= sizeof(flashdata_t)) {
      flashWriteData(address, *(const flashdata_t*)buffer);
      address += sizeof(flashdata_t);
      buffer += sizeof(flashdata_t);
      size -= sizeof(flashdata_t);
    }
    /* Lock flash again */
    flash_lock();
#elif defined(STM32H7)
    if (address <= 0x080FFFFF && FLASH_END >= 0x081FFFFF) { // 2MB devices
      /* Unlock flash for write access */
      if(flash_Unlock(1) == CH_FAILED) {
        return FLASH_RETURN_NO_PERMISSION;
      }
      /* Wait for any busy flags */
      flash_WaitWhileBusy(1);
      /* Setup parallelism before any program/erase */
      FLASH->CR1 &= ~FLASH_CR_PSIZE_MASK;
      FLASH->CR1 |= FLASH_CR_PSIZE_VALUE;
      while (size >= sizeof(flashdata_t)) {
        flashWriteData(address, *(const flashdata_t*)buffer);
        address += sizeof(flashdata_t);
        buffer += sizeof(flashdata_t);
        size -= sizeof(flashdata_t);
      }
      /* Lock flash again */
      flash_lock(1);
   } else if (address > 0x080FFFFF && FLASH_END >= 0x081FFFFF) {
      /* Unlock flash for write access */
      if(flash_Unlock(2) == CH_FAILED) {
        return FLASH_RETURN_NO_PERMISSION;
      }
      /* Wait for any busy flags */
      flash_WaitWhileBusy(2);
      /* Setup parallelism before any program/erase */
      FLASH->CR2 &= ~FLASH_CR_PSIZE_MASK;
      FLASH->CR2 |= FLASH_CR_PSIZE_VALUE;
      while (size >= sizeof(flashdata_t)) {
        flashWriteData(address, *(const flashdata_t*)buffer);
        address += sizeof(flashdata_t);
        buffer += sizeof(flashdata_t);
        size -= sizeof(flashdata_t);
      }
      /* Lock flash again */
      flash_lock(2);
   }
#endif

    chSysUnlock();

return FLASH_RETURN_SUCCESS;
}



int flash_Write_8bit(flashaddr_t address, const char* buffer, size_t size)
{

    chSysLock();

#if defined(STM32F7)
    /* Unlock flash for write access */
    if(flash_Unlock() == CH_FAILED) {
      return FLASH_RETURN_NO_PERMISSION;
    }
    /* Wait for any busy flags */
    flash_WaitWhileBusy();
    /* Setup parallelism before any program/erase */
    FLASH->CR &= ~FLASH_CR_PSIZE_MASK;
    FLASH->CR |= FLASH_CR_PSIZE_VALUE;
#elif defined(STM32H7)
    if (address <= 0x080FFFFF && FLASH_END >= 0x081FFFFF) { // 2MB devices
      /* Unlock flash for write access */
      if(flash_Unlock(1) == CH_FAILED) {
        return FLASH_RETURN_NO_PERMISSION;
      }
      /* Wait for any busy flags */
      flash_WaitWhileBusy(1);
      /* Setup parallelism before any program/erase */
      FLASH->CR1 &= ~FLASH_CR_PSIZE_MASK;
      FLASH->CR1 |= FLASH_CR_PSIZE_VALUE;
   } else if (address > 0x080FFFFF && FLASH_END >= 0x081FFFFF) {
      /* Unlock flash for write access */
      if(flash_Unlock(2) == CH_FAILED) {
        return FLASH_RETURN_NO_PERMISSION;
      }
      /* Wait for any busy flags */
      flash_WaitWhileBusy(2);
      /* Setup parallelism before any program/erase */
      FLASH->CR2 &= ~FLASH_CR_PSIZE_MASK;
      FLASH->CR2 |= FLASH_CR_PSIZE_VALUE;
   }
#endif
   /* Check if the flash address is correctly aligned */
   size_t alignOffset = address % sizeof(flashdata_t);
   if (alignOffset != 0) {
     /* Not aligned, thus we have to read the data in flash already present
     * and update them with buffer's data */
     /* Align the flash address correctly */
     flashaddr_t alignedFlashAddress = address - alignOffset;
     /* Read already present data */
     flashdata_t tmp = *(volatile flashdata_t*)alignedFlashAddress;
     /* Compute how much bytes one must update in the data read */
     size_t chunkSize = sizeof(flashdata_t) - alignOffset;
     if (chunkSize > size) {
         chunkSize = size; // this happens when both address and address + size are not aligned
     }
     /* Update the read data with buffer's data */
     memcpy((char*)&tmp + alignOffset, buffer, chunkSize);
     /* Write the new data in flash */
     flashWriteData(alignedFlashAddress, tmp);
     /* Advance */
     address += chunkSize;
     buffer += chunkSize;
     size -= chunkSize;
   }

   /* Now, address is correctly aligned. One can copy data directly from
   * buffer's data to flash memory until the size of the data remaining to be
   * copied requires special treatment. */
   while (size >= sizeof(flashdata_t)) {
     flashWriteData(address, *(const flashdata_t*)buffer);
     address += sizeof(flashdata_t);
     buffer += sizeof(flashdata_t);
     size -= sizeof(flashdata_t);
   }

   // Now the remaining data are LESS THAN THE SELECTED WRITE SIZE sizeof(flashdata_t).
   // To address this we will copy the remaining bytes to a full flashdata_t sized buffer
   // and then write this buffer to flash memory. */
   if (size > 0) {
     flashdata_t tmp = *(volatile flashdata_t*)address;
     memcpy(&tmp, buffer, size);
     flashWriteData(address, tmp);
   }

#if defined(STM32F7)
   /* Lock flash again */
    flash_lock();
#elif defined(STM32H7)
    if (address <= 0x080FFFFF && FLASH_END >= 0x081FFFFF) { // 2MB devices
      /* Lock flash again */
      flash_lock(1);
   } else if (address > 0x080FFFFF && FLASH_END >= 0x081FFFFF) {
      /* Lock flash again */
      flash_lock(2);
   }
#endif

    chSysUnlock();

return FLASH_RETURN_SUCCESS;
}


#if defined(STM32F7)
bool move_flash_boot_address(uint16_t flash_boot_address)
{
    // 88 = sector4 on ICTM interfase (0x00220000) and 2004 = sector4 in AXIM interface (0x08020000).
    // Whatever hex address you need to move the origin to just shft the address 14 bits to the right.
    chSysLock();
    flash_WaitWhileBusy();
    if ((FLASH->OPTCR1 & 0x0000FFFF) == flash_boot_address) {
      /* Write magic unlock sequence */
      FLASH->OPTKEYR = 0x08192A3B;
      FLASH->OPTKEYR = 0x4C5D6E7F;
      flash_WaitWhileBusy();
      FLASH->OPTCR1 = (FLASH->OPTCR1 & 0xFFFF0000)+flash_boot_address;
      //flash_WaitWhileBusy();
      FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT;
      flash_WaitWhileBusy();
      /* Check if unlock is really needed */
      if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->CR |= FLASH_CR_LOCK;
      }
      flash_WaitWhileBusy();
    } else {
      chSysUnlock();
      return(CH_FAILED);
    }
    chSysUnlock();

return(CH_SUCCESS);
}
#endif

