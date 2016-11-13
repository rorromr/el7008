#ifndef OBJECT_PSD_PCL_UTIL_H
#define OBJECT_PSD_PCL_UTIL_H

#include <stdint.h>
#include <stdlib.h>

namespace pcl_util
{

  /**
   * @enum Color
   * @brief Point Cloud colors
   */
  enum Color
  {
    BLACK,     /**< Black color. */
    BLUE,      /**< Blue color. */
    BROWN,     /**< Brown color. */
    CYAN,      /**< Cyan color. */
    DARK_GREY, /**< Dark grey color. */
    GREEN,     /**< Green. */
    GREY,      /**< Grey color. */
    LIME_GREEN,/**< Lime green color. */
    MAGENTA,   /**< Magenta color. */
    ORANGE,    /**< Orange color. */
    PINK,      /**< Pink color. */
    PURPLE,    /**< Purple color. */
    RED,       /**< Red color. */
    WHITE,     /**< White color. */
    YELLOW     /**< Yellow color. */
  };

  /**
   * @brief Get float color representation
   * @param color Target color
   * @return Color in float format
   */
  float getColor(const Color& color)
  {
    uint32_t base;
    switch (color)
    {
      case BLACK:
        base = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
        break;
      case BLUE:
        base = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)255);
        break;
      case BROWN:
        base = ((uint32_t)0 << 16 | (uint32_t)134 << 8 | (uint32_t)238);
        break;
      case CYAN:
        base = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
        break;
      case DARK_GREY:
        base = ((uint32_t)85 << 16 | (uint32_t)85 << 16 | (uint32_t)85);
        break;
      case GREEN:
        base = ((uint32_t)0 << 16 | (uint32_t)128 << 16 | (uint32_t)0);
        break;
      case GREY:
        base = ((uint32_t)128 << 16 | (uint32_t)128 << 8 | (uint32_t)128);
        break;
      case LIME_GREEN:
        base = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
        break;
      case MAGENTA:
        base = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)255);
        break;
      case ORANGE:
        base = ((uint32_t)255 << 16 | (uint32_t)165 << 8 | (uint32_t)0);
        break;
      case PINK:
        base = ((uint32_t)255 << 16 | (uint32_t)192 << 8 | (uint32_t)203);
        break;
      case PURPLE:
        base = ((uint32_t)128 << 16 | (uint32_t)0 << 8 | (uint32_t)128);
        break;
      case RED:
        base = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
        break;
      case WHITE:
        base = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
        break;
      case YELLOW:
        base = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
        break;
      default:
        base = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)255);
        break;
    }
    return *reinterpret_cast<float*>(&base);
  }

  /**
   * @brief Get float color representation
   * @param r Red component
   * @param g Green component
   * @param b Blue component
   * @return Color in float format
   */
  float getColor(uint8_t r, uint8_t g, uint8_t b)
  {
    uint32_t base = ((uint32_t)r << 16 | (uint32_t)b << 8 | (uint32_t)b);
    return *reinterpret_cast<float*>(&base);
  }

  /**
   * @brief Get random color from Color chart
   */
  float getRandomColor()
  {
	  return getColor(static_cast<Color>(std::rand() % YELLOW));
  }

}

#endif /* OBJECT_PSD_PCL_UTIL_H header guard */
