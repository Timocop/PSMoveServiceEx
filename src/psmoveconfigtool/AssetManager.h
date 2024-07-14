#ifndef ASSET_MANAGER_H
#define ASSET_MANAGER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
//#include <GL/gl.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "stb_truetype.h"

class TextureAsset
{
public:
    unsigned int texture_id;
    unsigned int texture_width;
    unsigned int texture_height;
    unsigned int texture_format;
    unsigned int buffer_format;

    TextureAsset()
        : texture_id(0)
        , texture_width(0)
        , texture_height(0)
        , texture_format(0)
        , buffer_format(0)
    {}
    ~TextureAsset()
    { dispose(); }

    bool init(unsigned int width, unsigned int height, unsigned int texture_format, unsigned int buffer_format, unsigned char *buffer);
    void copyBufferIntoTexture(const unsigned char *pixels);
    void dispose();
};

class FontAsset : public TextureAsset
{
public:
    float glyphPixelHeight;
    stbtt_bakedchar cdata[96]; // ASCII 32..126 is 95 glyphs

    FontAsset()
        : TextureAsset()
        , glyphPixelHeight(0.f)
    {}

    bool init(unsigned char *ttf_buffer, float pixel_height);
};

class AssetManager
{
public:
    AssetManager();
    ~AssetManager();

    bool init();
    void destroy();

	struct gl_model_asset
	{
		TextureAsset m_texture;
		std::vector<float> m_vert;
		std::vector<float> m_tex;
		std::vector<float> m_norm;
	};

    static AssetManager *getInstance()
	{
		return m_instance;
	}

    const FontAsset *getDefaultFont()
	{
		return &m_defaultFont;
	}

	// Models
	const gl_model_asset *getPS3EyeAsset()
	{
		return &m_ps3eye_assets;
	}

	const gl_model_asset *getPSMoveAsset()
	{
		return &m_psmove_assets;
	}

	const gl_model_asset *getPSMoveBulbAsset()
	{
		return &m_psmove_bulb_assets;
	}

	const gl_model_asset *getPSMorpheusAsset()
	{
		return &m_morpheus_assets;
	}

	const gl_model_asset *getPSMorpheusLedsAsset()
	{
		return &m_morpheus_leds_assets;
	}

	const gl_model_asset *getPSMorpheusBulbAsset()
	{
		return &m_morpheus_bulb_assets;
	}

	const gl_model_asset *getPSDualshockAsset()
	{
		return &m_dualshock_assets;
	}

	const gl_model_asset *getPSDualshockLedAsset()
	{
		return &m_dualshock_led_assets;
	}

	const gl_model_asset *getPSNavigationAsset()
	{
		return &m_psnavigation_assets;
	}

	// Icons
	const TextureAsset *getIconSettings()
	{
		return &m_icon_settings_asset;
	}

	const TextureAsset *getIconController()
	{
		return &m_icon_controller_asset;
	}

	const TextureAsset *getIconTracker()
	{
		return &m_icon_tracker_asset;
	}

	const TextureAsset *getIconHmd()
	{
		return &m_icon_hmd_asset;
	}

	const TextureAsset *getIconWaitDone()
	{
		return &m_icon_wait_done_asset;
	}

	const TextureAsset *getIconWaitEmpty()
	{
		return &m_icon_wait_empty_asset;
	}

	const TextureAsset *getIconWaitFull()
	{
		return &m_icon_wait_full_asset;
	}

	const TextureAsset *getIconWaitHalf()
	{
		return &m_icon_wait_half_asset;
	}

private:
    bool loadTexture(const char *filename, TextureAsset *textureAsset);
    bool loadFont(const char *filename, float pixelHeight, FontAsset *fontAsset);

	bool loadOBJ(
		const std::string & path,
		std::vector<float>& outVertices,
		std::vector<float>& outTexCoords,
		std::vector<float>& outnormals);

    // Utility Assets
	gl_model_asset m_ps3eye_assets;
	gl_model_asset m_psmove_assets;
	gl_model_asset m_psmove_bulb_assets;
	gl_model_asset m_morpheus_assets;
	gl_model_asset m_morpheus_leds_assets;
	gl_model_asset m_morpheus_bulb_assets;
	gl_model_asset m_dualshock_assets;
	gl_model_asset m_dualshock_led_assets;
	gl_model_asset m_psnavigation_assets;


    // Font Rendering
    FontAsset m_defaultFont;
	TextureAsset m_icon_settings_asset;
	TextureAsset m_icon_controller_asset;
	TextureAsset m_icon_tracker_asset;
	TextureAsset m_icon_hmd_asset;

	TextureAsset m_icon_wait_done_asset;
	TextureAsset m_icon_wait_empty_asset;
	TextureAsset m_icon_wait_full_asset;
	TextureAsset m_icon_wait_half_asset;

    static AssetManager *m_instance;
};

#endif // ASSET_MANAGER_H