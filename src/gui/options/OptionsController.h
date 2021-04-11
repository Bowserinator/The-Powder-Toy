#ifndef OPTIONSCONTROLLER_H_
#define OPTIONSCONTROLLER_H_
#include "Config.h"

#include <functional>

class GameModel;
class OptionsModel;
class ModOptionsView;
class OptionsView;
class OptionsController
{
	GameModel * gModel;
	OptionsView * view;
	ModOptionsView * mod_view;
	OptionsModel * model;
	std::function<void ()> onDone;
public:
	bool HasExited;
	OptionsController(GameModel * gModel_, bool modOptions = false, std::function<void ()> onDone = nullptr);
	void SetHeatSimulation(bool state);
	void SetAmbientHeatSimulation(bool state);
	void SetNewtonianGravity(bool state);
	void SetWaterEqualisation(bool state);
	void SetGravityMode(int gravityMode);
	void SetAirMode(int airMode);
	void SetEdgeMode(int edgeMode);
	void SetFullscreen(bool fullscreen);
	void SetAltFullscreen(bool altFullscreen);
	void SetForceIntegerScaling(bool forceIntegerScaling);
	void SetScale(int scale);
	void SetResizable(bool resizable);
	void SetFastQuit(bool fastquit);
	void SetDecoSpace(int decoSpace);
	void SetShowAvatars(bool showAvatars);
	void SetMouseClickrequired(bool mouseClickRequired);
	void SetIncludePressure(bool includePressure);
	void SetPerfectCircle(bool perfectCircle);
<<<<<<< HEAD
	void SetOppositeToolEnabled(bool oppositeTool);
	void SetSecretModShortcut(bool enabled);
	void SetCrosshairInBrush(bool enabled);
	void SetHollowBrushes(bool enabled);
	void SetAutoHideHUD(bool enabled);
	void SetDimGlowMode(bool enabled);
	void SetDrawingFrequencyLimit(int limit);
	void SetFasterRenderer(bool enabled);
	void SetSoundEnabled(bool enabled);

=======
	void SetMomentumScroll(bool momentumScroll);
	void SetAutoDrawLimit(bool autoDrawLimit);
	
>>>>>>> upstream/master
	void Exit();
	OptionsView * GetView();
	virtual ~OptionsController();
};

#endif /* OPTIONSCONTROLLER_H_ */
