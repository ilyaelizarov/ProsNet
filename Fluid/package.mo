within ProsNet;
package Fluid "Package with models for fluid flow systems"
    extends Modelica.Icons.Package;

  annotation (Icon(graphics={
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Polygon(points={{-70,26},{68,-44},{68,26},{2,-10},{-70,-42},{-70,26}}),
        Line(points={{2,42},{2,-10}}),
        Rectangle(
          extent={{-18,50},{22,42}},
          fillPattern=FillPattern.Solid)}), Documentation(info= "<html><head></head><body><p>This package contains components for fluid flow systems such as pumps, heat exchangers, valves and sensors. For other fluid flow models, see <a href=\"modelica://Modelica.Fluid\">Modelica.Fluid</a>. </p>
<p><br>Most of the models are copies from the package with the same name from the <a href=\"https://github.com/ibpsa/modelica-ibpsa\">IBPSA library</a>.</p>
</body></html>"));
end Fluid;