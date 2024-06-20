within ProsNet.Controls;
package GeneratorControlPackage
  model ControlGenerator
   model GeneratorControl
    // Define connectors
    connector RealInput = input Real;
    connector BooleanInput = input Boolean;
    connector BooleanOutput = output Boolean;

    // Define parameters
    parameter Real TStartCHP; // Temperature threshold to start the CHP
    parameter Real TStopCHP; // Temperature threshold to stop the CHP

    // Define input And output connectors

     Modelica.Blocks.Interfaces.RealInput TopTlayer
       annotation (Placement(transformation(extent={{-148,12},{-108,52}})));
     Modelica.Blocks.Interfaces.RealInput LowTlayer
       annotation (Placement(transformation(extent={{-148,-98},{-108,-58}})));
     Modelica.Blocks.Interfaces.BooleanInput UseChp
       annotation (Placement(transformation(extent={{-148,-38},{-108,2}})));
     Modelica.Blocks.Interfaces.BooleanOutput CHPON
       annotation (Placement(transformation(extent={{76,-10},{96,10}})));
    protected
    Boolean CHPon_internal; // Internal variable to hold the CHP state

   algorithm
    // Control logic for turning the CHP on
    if (TopTlayer < TStartCHP and UseChp) then
      CHPon_internal := true;
    end if;

    // Additional logic to turn the CHP off when lower layer temperature is too high
    if (LowTlayer > TStopCHP) then
      CHPon_internal := false;
    end if;

    // Assign the internal CHP state to the output
    CHPON := CHPon_internal;

   end GeneratorControl;

   model ElHeaterControl
    // Define connectors
    connector RealInput = input Real;
    connector BooleanInput = input Boolean;
    connector BooleanOutput = output Boolean;

    // Define parameters
    parameter Real deltaTEHon; // Temperature threshold to start the CHP
    parameter Real Tmax_TS; // Temperature threshold to stop the CHP

    // Define input And output connectors

     Modelica.Blocks.Interfaces.RealInput Tlayer5
       annotation (Placement(transformation(extent={{-148,12},{-108,52}})));
     Modelica.Blocks.Interfaces.BooleanInput Use
       annotation (Placement(transformation(extent={{-148,-38},{-108,2}})));
     Modelica.Blocks.Interfaces.BooleanOutput ON
       annotation (Placement(transformation(extent={{76,-10},{96,10}})));
    protected
    Boolean internal; // Internal variable to hold the CHP state

   algorithm
    // Control logic for turning the CHP on
    if (Tmax_TS - Tlayer5 > deltaTEHon and Use) then
      internal := true;
   else
      internal := false;
    end if;

    // Assign the internal CHP state to the output
    ON := internal;

   end ElHeaterControl;

   model HPControl
    // Define connectors
    connector RealInput = input Real;
    connector BooleanInput = input Boolean;
    connector BooleanOutput = output Boolean;

    // Define parameters
    parameter Real TStart; // Temperature threshold to start the
    parameter Real TStop; // Temperature threshold to stop the

    // Define input And output connectors

     Modelica.Blocks.Interfaces.RealInput TopTlayer
       annotation (Placement(transformation(extent={{-148,12},{-108,52}})));
     Modelica.Blocks.Interfaces.RealInput LowTlayer
       annotation (Placement(transformation(extent={{-148,-98},{-108,-58}})));
     Modelica.Blocks.Interfaces.BooleanInput Use
       annotation (Placement(transformation(extent={{-148,-38},{-108,2}})));
     Modelica.Blocks.Interfaces.BooleanOutput ON
       annotation (Placement(transformation(extent={{76,-10},{96,10}})));
    protected
    Boolean on_internal; // Internal variable to hold the  state

   algorithm
    // Control logic for turning the  on
    if (TopTlayer < TStart and Use) then
      on_internal := true;
    end if;

    // Additional logic to turn the  off when lower layer temperature is too high
    if (LowTlayer > TStop) then
      on_internal := false;
    end if;

    // Assign the internal  state to the output
    ON := on_internal;

   end HPControl;

   model SolarThermalControl
    // Define connectors
    connector RealInput = input Real;
    connector BooleanInput = input Boolean;
    connector BooleanOutput = output Boolean;

    // Define parameters
    parameter Real deltaTonST; // Temperature threshold to start the
    parameter Real TCollectorST; // Temperature threshold to stop the

    // Define input And output connectors

     Modelica.Blocks.Interfaces.RealInput Tlayer5
       annotation (Placement(transformation(extent={{-148,12},{-108,52}})));
     Modelica.Blocks.Interfaces.BooleanInput Use
       annotation (Placement(transformation(extent={{-148,-38},{-108,2}})));
     Modelica.Blocks.Interfaces.BooleanOutput ON
       annotation (Placement(transformation(extent={{76,-10},{96,10}})));
    protected
    Boolean internal; // Internal variable to hold the  state

   algorithm
    // Control logic for turning the on
    if (TCollectorST - Tlayer5 > deltaTonST and Use) then
      internal := true;
   else
      internal := false;
    end if;

    // Assign the internal state to the output
    ON := internal;

   end SolarThermalControl;
   annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end ControlGenerator;
end GeneratorControlPackage;
