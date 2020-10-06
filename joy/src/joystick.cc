/**
 * @file joystick.c
 * @brief Joystick Interface Source
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_joystick.h>
#include <SDL2/SDL_gamecontroller.h>

#include "core/include/oscc.h"
#include "joy/include/joystick.h"

/**
 * @brief Button press debounce delay. [microseconds]
 */
#define BUTTON_PRESSED_DELAY 5000

/**
 * @brief Invalid \ref joystick_device_s.controller value
 */
#define JOYSTICK_DEVICE_CONTROLLER_INVALID NULL

/**
 * @brief Joystick Identifier Data
 */
#define JOYSTICK_ID_DATA_SIZE 16

/**
 * @brief Joystick Description String
 */
#define JOYSTICK_ID_STRING_SIZE 64

typedef struct
{
  unsigned char data[JOYSTICK_ID_DATA_SIZE];
  char ascii_string[JOYSTICK_ID_STRING_SIZE];
} joystick_guid_s;

typedef struct
{
  SDL_GameController* controller;
  SDL_Haptic* haptic;
  joystick_guid_s* guid;
} joystick_device_data_s;

static joystick_guid_s joystick_guid;
static joystick_device_data_s joystick_data = {.controller = NULL, 
                                               .haptic = NULL, 
                                               .guid = &joystick_guid };
static joystick_device_data_s* joystick = NULL;

static oscc_result_t joystick_init_subsystem()
{
  oscc_result_t ret = OSCC_ERROR;
  if (joystick == NULL)
  {
    int init_result = SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC);
    ret = OSCC_OK;
    if (init_result < 0)
    {
      printf("OSCC_ERROR: SDL_Init - %s\n", SDL_GetError());
      ret = OSCC_ERROR;
    }
  }
  return ret;
}

static oscc_result_t joystick_get_guid_at_index(int device_index)
{
  oscc_result_t result = OSCC_ERROR;
  if (joystick != NULL)
  {
    result = OSCC_OK;
    const SDL_JoystickGUID m_guid = SDL_JoystickGetDeviceGUID(device_index);
    memcpy(joystick_guid.data, m_guid.data, sizeof(m_guid.data));
    memset(joystick_guid.ascii_string, 0,
           sizeof(joystick_guid.ascii_string) );
    SDL_JoystickGetGUIDString(m_guid,
                              joystick_guid.ascii_string,
                              sizeof(joystick_guid.ascii_string) );
  }
  return result;
}

static int joystick_get_num_devices()
{
  int num_joysticks = OSCC_ERROR;
  if (joystick != NULL)
  {
    num_joysticks = SDL_NumJoysticks();
    if (num_joysticks < 0)
    {
      printf("OSCC_ERROR: SDL_NumJoysticks - %s\n", SDL_GetError());
      num_joysticks = OSCC_ERROR;
    }
  }
  return  num_joysticks;
}

oscc_result_t joystick_init()
{
  oscc_result_t result = OSCC_OK;
  result = joystick_init_subsystem();
  if (result == OSCC_ERROR)
    printf("init subsystem error\n");
  else
  {
    joystick = &joystick_data;
    joystick->controller = JOYSTICK_DEVICE_CONTROLLER_INVALID;
    const int num_joysticks = joystick_get_num_devices();
    if (num_joysticks > 0)
    {
      unsigned long device_index = 0;
      result = joystick_get_guid_at_index(device_index);
      if (result == OSCC_OK)
      {
        printf("Found %d devices -- connecting to device at system index %lu - GUID: %s\n",
               num_joysticks,
               device_index,
               joystick_guid.ascii_string                                                   );
        result = joystick_open(device_index);
      }
    }
    else
      printf( "No joystick/devices available on the host\n" );
  }
  return result;
}

oscc_result_t joystick_open(int device_index)
{
  oscc_result_t result = OSCC_ERROR;
  if (joystick != NULL)
  {
    joystick->controller = SDL_GameControllerOpen(device_index);

    if (joystick->controller == JOYSTICK_DEVICE_CONTROLLER_INVALID)
      printf("OSCC_ERROR: SDL_JoystickOpen - %s\n", SDL_GetError());
    else
    {
      result = OSCC_OK;
      const SDL_JoystickGUID m_guid = SDL_JoystickGetGUID(SDL_GameControllerGetJoystick(joystick->controller));
      memcpy(joystick_guid.data, m_guid.data, sizeof( m_guid.data));
      memset(joystick_guid.ascii_string, 0,
             sizeof(joystick_guid.ascii_string) );
      SDL_JoystickGetGUIDString(m_guid,
                                joystick_guid.ascii_string,
                                sizeof(joystick_guid.ascii_string) );
      joystick->haptic = SDL_HapticOpenFromJoystick(SDL_GameControllerGetJoystick(joystick->controller));
      if (SDL_HapticRumbleInit(joystick->haptic) != 0)
        SDL_HapticClose( joystick->haptic );
    }
  }
  return result;
}

void joystick_close()
{
  if (joystick != NULL)
  {
    if (joystick->controller != JOYSTICK_DEVICE_CONTROLLER_INVALID)
    {
      if (SDL_GameControllerGetAttached(joystick->controller) == SDL_TRUE)
      {
        if (joystick->haptic)
          SDL_HapticClose(joystick->haptic);
        SDL_GameControllerClose(joystick->controller);
      }
      joystick->controller = JOYSTICK_DEVICE_CONTROLLER_INVALID;
    }
    joystick = NULL;
  }
  // Release the joystick subsystem
  SDL_Quit();
}

oscc_result_t joystick_update()
{
  oscc_result_t result = OSCC_ERROR;
  if (joystick != NULL)
  {
    if (joystick->controller != JOYSTICK_DEVICE_CONTROLLER_INVALID)
    {
      SDL_GameControllerUpdate();
      if (SDL_GameControllerGetAttached(joystick->controller) == SDL_FALSE)
        printf("SDL_GameControllerGetAttached - device not attached\n");
      else
        result = OSCC_OK;
    }
  }
  return result;
}

oscc_result_t joystick_get_axis(SDL_GameControllerAxis axis_index, int* const position)
{
  oscc_result_t result = OSCC_ERROR;
  if (joystick!=NULL && position!=NULL)
  {
    result = OSCC_OK;
    const Sint16 pos = SDL_GameControllerGetAxis(joystick->controller,
                                                 axis_index           );
    *position = (int)pos;
  }
  return result;
}

oscc_result_t joystick_get_button(SDL_GameControllerButton button_index, unsigned int* const button_state)
{
  oscc_result_t result = OSCC_ERROR;
  if (joystick!=NULL && button_state!=NULL)
  {
    result = OSCC_OK;
    const Uint8 m_state = SDL_GameControllerGetButton(joystick->controller,
                                                      button_index          );
    if (m_state == 1)
    {
      *button_state = JOYSTICK_BUTTON_STATE_PRESSED;

      if (joystick->haptic)
        SDL_HapticRumblePlay(joystick->haptic, 1.0f, 100);

      (void)usleep(BUTTON_PRESSED_DELAY);
    }
    else
      *button_state = JOYSTICK_BUTTON_STATE_NOT_PRESSED;
  }
  return result;
}
