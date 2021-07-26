within ;
model controller_simple

  import      Modelica.Units.SI;
  import T_AbsZeroDegC = Modelica.Constants.T_zero;

  parameter SI.Temperature T_prim_hot_des(min=277) = -T_AbsZeroDegC + 70
    "desired temperature supply primary side";
  parameter SI.Temperature T_prim_cold_des(min=277) = -T_AbsZeroDegC + 35
    "desired temperature return primary side";
  parameter SI.Temperature T_sec_hot_des(min=277) = -T_AbsZeroDegC + 60
    "desired temperature supply secondary side";
  parameter SI.Temperature T_sec_cold_des(min=277) = -T_AbsZeroDegC + 45
    "desired temperature return primary side";
  parameter SI.Temperature T_prim_cold_max(min=277) = -T_AbsZeroDegC + 40
    "desired temperature return primary side";
  parameter SI.Temperature T_sec_cold_max(min=277) = -T_AbsZeroDegC + 50
    "desired temperature return primary side";
  parameter SI.Temperature T_prim_hot_min(min=277) = -T_AbsZeroDegC + 65
    "desired temperature return primary side";
  parameter SI.Temperature T_sec_hot_min(min=277) = -T_AbsZeroDegC + 55
    "desired temperature return primary side";
  //parameter SI.TemperatureDifference Delta_T_transfer_Drive(min=0) = 5
  //"desired temperature return primary side";

  Modelica.Blocks.Interfaces.IntegerInput prosumer_mode(min= -1, max= 1)
    "prosumer mode (-1 consumption, 0 idle, 1 production)"
    annotation (Placement(transformation(extent={{-120,-20},{-80,20}})));
  Modelica.Blocks.Interfaces.IntegerOutput pi(min=0, max=1)
    "prosumer participation"    annotation (Placement(transformation(extent={{90,50},{110,70}})));
  Modelica.Blocks.Interfaces.IntegerOutput mu(min=-1, max=1)
    "prosumer mode (-1 for consumption, 1 for production)"    annotation (Placement(transformation(extent={{90,30},{110,50}})));
  Modelica.Blocks.Interfaces.RealOutput u(min= 0, max= 1)
    "pump control signal for production mode"    annotation (Placement(transformation(extent={{90,-30},{110,-10}})));
  Modelica.Blocks.Interfaces.RealOutput kappa(min= 0, max= 1)
    "valve control signal for consumption mode"    annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
  Modelica.Blocks.Interfaces.RealInput T_sec_hot(unit = "K", displayUnit="degC",
    min=277) "current temperature hot level secondary side"      annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-40,100})));
  Modelica.Blocks.Interfaces.RealInput T_sec_cold(unit = "K", displayUnit="degC",
  min=277) "current temperature cold  level secondary side"    annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={20,100})));
  Modelica.Blocks.Interfaces.RealInput T_prim_hot(unit = "K", displayUnit="degC",
  min=277) "current temperature hot level primary side"        annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-40,-100})));
  Modelica.Blocks.Interfaces.RealInput T_prim_cold(unit = "K", displayUnit="degC",
  min=277)  "current temperature cold level primary side"       annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={20,-100})));
  Modelica.Blocks.Continuous.LimPID PID_production(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.1,
    Ti=1,
    Td=0.1,
    yMax=1,                                                 yMin=0,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    y_start=1)
    annotation (Placement(transformation(extent={{-50,20},{-30,40}})));
  Modelica.Blocks.Continuous.LimPID PID_consumption(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.1,
    Ti=5,
    Td=0.1,
    yMax=1,                                                  yMin=0,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    y_start=1)
    annotation (Placement(transformation(extent={{-50,-38},{-30,-18}})));
equation
  // connect PID controllers
  PID_consumption.u_s = T_sec_hot_des;
  connect( T_sec_hot, PID_consumption.u_m);
  PID_production.u_s = T_prim_hot_des;
  connect( T_prim_hot, PID_production.u_m);

  // different cases for different prosumer modes
  if prosumer_mode == -1 then // consumption mode
    // determine mu and pi for consumption mode
    pi = 1;
    mu= -1;
    u = 0;
    kappa = PID_consumption.y;
  elseif prosumer_mode == 1 then // production mode
    // determine mu and pi for production mode
    pi = 1;
    mu = 1;
    kappa = 0;
    u = PID_production.y;
  else // idle mode
    // determine mu and pi for idle mode
    pi = 0;
    mu = -1;
    u = 0;
    kappa = 0;
  end if;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="4.0.0")),
    version="1",
    conversion(noneFromVersion=""));
end controller_simple;
