#include "flash.h"

uint32_t PAGEError = 0;

//Variable used for Erase procedure
static FLASH_EraseInitTypeDef EraseInitStruct;

uint32_t erase_flash_page(void)
{
    //Erase the user Flash area
    //(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR)

    //Fill EraseInit structure
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

    HAL_FLASH_Unlock();

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
    {
        // Error occurred while page erase.
        // User can add here some code to deal with this error.
        // PAGEError will contain the faulty page and then to know the code error on this page,
        // user can call function 'HAL_FLASH_GetError()'

        HAL_FLASH_Lock();
        return HAL_FLASH_GetError();
    }
    else
    {
        HAL_FLASH_Lock();
        return HAL_OK;
    }
}

uint32_t write_flash_vars(float* data, uint8_t length)
{
    uint32_t Address = FLASH_USER_START_ADDR;
    __IO uint32_t data32;

    HAL_FLASH_Unlock();
    while (length > 0)
    {
        data32 = *(__IO uint32_t *) data;

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, data32) == HAL_OK)
        {
            Address += 4;
            data++;
            length--;
        }
        else
        {
            return HAL_FLASH_GetError();
        }
    }
    HAL_FLASH_Lock();
    return HAL_OK;
}

void read_flash_vars(float data[], uint8_t length)
{
    uint32_t Address = FLASH_USER_START_ADDR;
    uint8_t i;

    for (i=0; i<length; i++)
    {
            data[i] = *(float*) Address;
            Address += 4;
    }
}

