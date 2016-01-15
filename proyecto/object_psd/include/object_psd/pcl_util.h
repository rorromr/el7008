#ifndef OBJECT_PSD_PCL_UTIL_H_
#define OBJECT_PSD_PCL_UTIL_H_

#include <stdint.h>

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
    }
    return *reinterpret_cast<float*>(&base);
  }

}


#endif /* OBJECT_PSD_PCL_UTIL_H_ */
