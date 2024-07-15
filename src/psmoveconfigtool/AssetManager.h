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

#include <imgui.h>

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

	ImTextureID getImTextureId() const 
	{
		return reinterpret_cast<ImTextureID>(static_cast<intptr_t>(texture_id));
	}

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

	const TextureAsset *getIconUpdate()
	{
		return &m_icon_update_asset;
	}

	const TextureAsset *getIconUpdate2()
	{
		return &m_icon_update2_asset;
	}

	const TextureAsset *getIconWarning()
	{
		return &m_icon_warning_asset;
	}

	const TextureAsset *getIconExclamation()
	{
		return &m_icon_exclamation_asset;
	}

	const TextureAsset *getIconUsb()
	{
		return &m_icon_usb_asset;
	}

	const TextureAsset *getIconBluetooth()
	{
		return &m_icon_bluetooth_asset;
	}

	const TextureAsset *getIconConnect()
	{
		return &m_icon_connect_asset;
	}

	const TextureAsset *getIconBan()
	{
		return &m_icon_ban_asset;
	}

	const TextureAsset *getIconShield()
	{
		return &m_icon_shield_asset;
	}

	const TextureAsset *getIconSearch()
	{
		return &m_icon_search_asset;
	}

	const TextureAsset *getIconCheck()
	{
		return &m_icon_check_asset;
	}

	const TextureAsset *getIconClose()
	{
		return &m_icon_close_asset;
	}

	const TextureAsset *getIconLeft()
	{
		return &m_icon_left_asset;
	}

	const TextureAsset *getIconRight()
	{
		return &m_icon_right_asset;
	}

	const TextureAsset *getIconTarget()
	{
		return &m_icon_target_asset;
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
	TextureAsset m_icon_update_asset;
	TextureAsset m_icon_update2_asset;
	TextureAsset m_icon_warning_asset;
	TextureAsset m_icon_exclamation_asset;
	TextureAsset m_icon_usb_asset;
	TextureAsset m_icon_bluetooth_asset;
	TextureAsset m_icon_connect_asset;
	TextureAsset m_icon_ban_asset;
	TextureAsset m_icon_shield_asset;
	TextureAsset m_icon_search_asset;
	TextureAsset m_icon_check_asset;
	TextureAsset m_icon_close_asset;
	TextureAsset m_icon_left_asset;
	TextureAsset m_icon_right_asset;
	TextureAsset m_icon_target_asset;

    static AssetManager *m_instance;
};

#endif // ASSET_MANAGER_H