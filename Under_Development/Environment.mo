within ProsNet.Under_Development;
package Environment "Environment"
  model Environment "Environment with user defined weather profiles"
   import Modelica.Utilities.Strings;
   import GreenCity.Utilities.WeatherData.Cities.WeatherConditions;
   import GreenCity.Utilities.WeatherData.Cities.WeatherConditionsTRY;
   import GreenCity.Environment.*;
   import GreenCity.*;
   GreenCity.Interfaces.Environment.EnvironmentConditions EnvironmentConditions "Enviromnent Conditions Connection" annotation(Placement(
    transformation(extent={{105,0},{125,20}}),
    iconTransformation(extent={{-160,40},{-140,60}})));
  protected
    function initTimeBaseUnix "initTimeBaseUnix"
     input Real tStart;
     output Real HourOfDay;
     output Real DayOfWeek;
     output Real DayOfMonth;
     output Real MonthOfYear;
     output Real DayOfYear;
     output Real Year;
     output Integer DSTEnableDay;
     output Integer DSTDisableDay;
    protected
      Integer auxYear;
      Integer auxMonth;
      Integer leapYearCount;
      Boolean thisLeapYear;
      Integer auxDayOfWeek1stJAN;
      Integer auxDayOfWeek;
      Integer thisYear;
      Integer DayCounter;
      Integer LastSuOnMarch;
      Integer LastSuOnOctober;
      Integer auxDSTDayOfWeek;
      parameter Integer DaysLeap[12]={31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      parameter Integer DaysNoLeap[12]={31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      parameter Integer InitDayOfWeek1969=3;
    algorithm
      auxYear:=integer(floor((tStart+3600)/(3600*24*365)));
      leapYearCount:=0;
      for i in 1970:(1970+auxYear) loop
       if ((not (mod(floor(i),4)>0)) or not (mod(floor(i),400)>0)) then
        leapYearCount:=leapYearCount+1;
       end if;
      end for;

      if (tStart+3600)-(auxYear*3600*24*365+(leapYearCount-1)*3600*24)>=0 then
       thisYear:=auxYear+1970;
      else
       thisYear:=auxYear+1970-1;
      end if;

      auxDayOfWeek1stJAN:=InitDayOfWeek1969;
      for i in 1970:thisYear loop
       if ((not (mod(floor(i),4)>0)) or not (mod(floor(i),400)>0)) then
        auxDayOfWeek1stJAN:=auxDayOfWeek1stJAN+2;
       else
        auxDayOfWeek1stJAN:=auxDayOfWeek1stJAN+1;
       end if;

       if auxDayOfWeek1stJAN>8 then
        auxDayOfWeek1stJAN:=2;
       elseif auxDayOfWeek1stJAN>7 then
        auxDayOfWeek1stJAN:=1;
       else
        auxDayOfWeek1stJAN:=auxDayOfWeek1stJAN;
       end if;
      end for;

      if ((not (mod(floor(thisYear),4)>0)) or not (mod(floor(thisYear),400)>0)) then
       thisLeapYear:=true;
      else
       thisLeapYear:=false;
      end if;

      if thisLeapYear then
       if auxDayOfWeek1stJAN>1 then
        auxDayOfWeek1stJAN:=auxDayOfWeek1stJAN-1;
       else
        auxDayOfWeek1stJAN:=7;
       end if;
      else
       auxDayOfWeek1stJAN:=auxDayOfWeek1stJAN;
      end if;

      if thisLeapYear then

       DayOfYear:=(tStart+3600-(thisYear-1970)*3600*24*365-(leapYearCount-1)*3600*24)/(24*3600);

       auxDSTDayOfWeek:=auxDayOfWeek1stJAN;
       LastSuOnMarch:=0;
       LastSuOnOctober:=0;
       DayCounter:=1;
       for i in 1:365 loop
        auxDSTDayOfWeek:=auxDSTDayOfWeek+1;
        DayCounter:=DayCounter+1;

        if auxDSTDayOfWeek>7 then
         auxDSTDayOfWeek:=1;
        end if;

        if auxDSTDayOfWeek==7 and DayCounter<sumOfDays(DaysLeap,1,3) then
         LastSuOnMarch:=DayCounter;
        end if;

        if auxDSTDayOfWeek==7 and DayCounter<sumOfDays(DaysLeap,1,10) then
         LastSuOnOctober:=DayCounter;
        end if;
       end for;

       if DayOfYear>=335 then
        auxMonth:=12;
       elseif DayOfYear>=305 then
        auxMonth:=11;
       elseif DayOfYear>=274-1/24 then
        auxMonth:=10;
       elseif DayOfYear>=244-1/24 then
        auxMonth:=9;
       elseif DayOfYear>=213-1/24 then
        auxMonth:=8;
       elseif DayOfYear>=182-1/24 then
        auxMonth:=7;
       elseif DayOfYear>=152-1/24 then
        auxMonth:=6;
       elseif DayOfYear>=121-1/24 then
        auxMonth:=5;
       elseif DayOfYear>=91-1/24 then
        auxMonth:=4;
       elseif DayOfYear>=60 then
        auxMonth:=3;
       elseif DayOfYear>=31 then
        auxMonth:=2;
       else
        auxMonth:=1;
       end if;

       if auxMonth>1 then
        if DayOfYear>LastSuOnMarch+2/24 and DayOfYear<LastSuOnOctober+3/24 then
         MonthOfYear:=auxMonth+(DayOfYear-sumOfDays(DaysLeap,1,auxMonth-1))/DaysLeap[auxMonth]+1/(24*DaysLeap[auxMonth]);
         DayOfMonth:=DayOfYear+1/24-sumOfDays(DaysLeap,1,auxMonth-1)+1;
        else
         MonthOfYear:=auxMonth+(DayOfYear-sumOfDays(DaysLeap,1,auxMonth-1))/DaysLeap[auxMonth];
         DayOfMonth:=DayOfYear-sumOfDays(DaysLeap,1,auxMonth-1)+1;
        end if;
       else
        if DayOfYear>LastSuOnMarch+2/24 and DayOfYear<LastSuOnOctober+3/24 then
         MonthOfYear:=auxMonth+DayOfYear/DaysLeap[auxMonth]+1/(24*DaysLeap[auxMonth]);
         DayOfMonth:=DayOfYear+1/24+1;
        else
         MonthOfYear:=auxMonth+DayOfYear/DaysLeap[auxMonth];
         DayOfMonth:=DayOfYear+1;
        end if;
       end if;

       Year:=thisYear+DayOfYear/sumOfDays(DaysLeap,1,12);
      else

       DayOfYear:=(tStart+3600-(thisYear-1970)*3600*24*365-leapYearCount*3600*24)/(24*3600);

       auxDSTDayOfWeek:=auxDayOfWeek1stJAN;
       auxDayOfWeek:=auxDayOfWeek1stJAN;
       LastSuOnMarch:=0;
       LastSuOnOctober:=0;
       DayCounter:=1;
       for i in 1:364 loop
        auxDSTDayOfWeek:=auxDSTDayOfWeek+1;
        DayCounter:=DayCounter+1;

        if auxDSTDayOfWeek>7 then
         auxDSTDayOfWeek:=1;
        end if;

        if auxDSTDayOfWeek==7 and DayCounter<sumOfDays(DaysNoLeap,1,3) then
         LastSuOnMarch:=DayCounter;
        end if;

        if auxDSTDayOfWeek==7 and DayCounter<sumOfDays(DaysNoLeap,1,10) then
         LastSuOnOctober:=DayCounter;
        end if;
       end for;

       if DayOfYear>=334 then
        auxMonth:=12;
       elseif DayOfYear>=304 then
        auxMonth:=11;
       elseif DayOfYear>=273-1/24 then
        auxMonth:=10;
       elseif DayOfYear>=243-1/24 then
        auxMonth:=9;
       elseif DayOfYear>=212-1/24 then
        auxMonth:=8;
       elseif DayOfYear>=181-1/24 then
        auxMonth:=7;
       elseif DayOfYear>=151-1/24 then
        auxMonth:=6;
       elseif DayOfYear>=120-1/24 then
        auxMonth:=5;
       elseif DayOfYear>=90-1/24 then
        auxMonth:=4;
       elseif DayOfYear>=59 then
        auxMonth:=3;
       elseif DayOfYear>=31 then
        auxMonth:=2;
       else
        auxMonth:=1;
       end if;

       if auxMonth>1 then
        if DayOfYear>LastSuOnMarch+2/24 and DayOfYear<LastSuOnOctober+3/24 then
         MonthOfYear:=auxMonth+(DayOfYear-sumOfDays(DaysNoLeap,1,auxMonth-1))/DaysNoLeap[auxMonth]+1/(24*DaysNoLeap[auxMonth]);
         DayOfMonth:=DayOfYear+1/24-sumOfDays(DaysNoLeap,1,auxMonth-1)+1;
        else
         MonthOfYear:=auxMonth+(DayOfYear-sumOfDays(DaysNoLeap,1,auxMonth-1))/DaysNoLeap[auxMonth];
         DayOfMonth:=DayOfYear-sumOfDays(DaysNoLeap,1,auxMonth-1)+1;
        end if;
       else
        if DayOfYear>LastSuOnMarch+2/24 and DayOfYear<LastSuOnOctober+3/24 then
         MonthOfYear:=auxMonth+DayOfYear/DaysNoLeap[auxMonth]+1/(24*DaysNoLeap[auxMonth]);
         DayOfMonth:=DayOfYear+1/24+1;
        else
         MonthOfYear:=auxMonth+DayOfYear/DaysNoLeap[auxMonth];
         DayOfMonth:=DayOfYear+1;
        end if;
       end if;

       Year:=thisYear+DayOfYear/sumOfDays(DaysNoLeap,1,12);
      end if;

      HourOfDay:=(DayOfMonth-integer(floor(DayOfMonth)))*24;

      auxDayOfWeek:=auxDayOfWeek1stJAN;
      for i in 1:integer(DayOfYear) loop
       auxDayOfWeek:=auxDayOfWeek+1;

       if auxDayOfWeek>7 then
        auxDayOfWeek:=1;
       end if;
      end for;

      if DayOfYear>=LastSuOnMarch and DayOfYear<LastSuOnOctober and DayOfYear-integer(floor(DayOfYear))>23/24 then
       if auxDayOfWeek==7 then
        auxDayOfWeek:=1;
       else
        auxDayOfWeek:=auxDayOfWeek+1;
       end if;
      end if;

      DSTEnableDay:=LastSuOnMarch;
      DSTDisableDay:=LastSuOnOctober;
      DayOfWeek:=auxDayOfWeek;
     annotation (Error,
      typename="ClassSymbolInfo");
    end initTimeBaseUnix;

    function getAzimuth "getAzimuth"
     input Real actRealHour;
     input Real SolAlt;
     input Real lat;
     input Real delta;
     output Real azimuth;
    protected
      Integer sign;
    algorithm
      if actRealHour<=12 then
       sign:=-1;
      else
       sign:=1;
      end if;
      azimuth:=pi+sign*acos(min(max((sin(SolAlt)*sin(lat)-sin(delta))/(cos(SolAlt)*cos(lat)),-1),1));
    end getAzimuth;

    function sumOfDays
     input Integer u[:];
     input Integer first=1;
     input Integer last=12;
     output Integer sum;
    algorithm
      sum:=0;
      for i in first:last loop
       sum:=sum+u[i];
      end for;
     annotation(__esi_Impure=false);
    end sumOfDays;

    function getVectorValue "getVectorValue"
     input Integer u[:];
     input Real indexReal;
     output Real value;
    protected
      Integer index;
      parameter Integer first=1;
      parameter Integer last=12;
    algorithm
      index:=min(max(integer(floor(indexReal)),first),last);
      value:=u[index];
    end getVectorValue;

    function angleRadian "angleRadian = angle(°) to rad"
     input Real angleDeg;
     output Real angleRad;
    algorithm
      angleRad:=angleDeg/180*pi;
     annotation (
      __esi_Impure=false,
        Error,
       typename="ClassSymbolInfo");
    end angleRadian;

    function normalVector "normalVector"
     input Real alpha_inc;
     input Real alpha_or;
     output Real vector[3];
    algorithm
      vector[1]:=sin(alpha_inc)*cos(alpha_or);
      vector[2]:=sin(alpha_inc)*sin(alpha_or);
      vector[3]:=cos(alpha_inc);
     annotation(__esi_Impure=false);
    end normalVector;

    function diffAngle
     input Real vector1[3];
     input Real vector2[3];
     output Real angle;
    protected
      Real value;
      Real minMax;
    algorithm
      value:=(vector1[1]*vector2[1]+vector1[2]*vector2[2]+vector1[3]*vector2[3])/(sqrt(vector1[1]*vector1[1]+vector1[2]*vector1[2]+vector1[3]*vector1[3])*sqrt(vector2[1]*vector2[1]+vector2[2]*vector2[2]+vector2[3]*vector2[3]));
      minMax:=max(min(value,1),-1);
      angle:=acos(minMax);
     annotation(__esi_Impure=false);
    end diffAngle;

    function normalVector2D
     input Real alpha;
     output Real vector2D[2];
    algorithm
      vector2D[1]:=cos(alpha);
      vector2D[2]:=sin(alpha);
     annotation(__esi_Impure=false);
    end normalVector2D;
  public
    Modelica.Blocks.Sources.RealExpression TimeBase(y=UnixTimeInit + time) "Set output signal to a time varying Real expression" annotation(Placement(transformation(extent={{-60,65},{-40,85}})));
    Modelica.Blocks.Tables.CombiTable1D WeatherData(
     tableOnFile=true,
     tableName="Weather",
     fileName=Filename,
     columns={2,3,4,5,6,7,8}) "0 - time[s]
; 1 - Ambient temperature [°C]
; 2 - 7 day average temperature [°C]; 3 - Solar direct radiation in radiation direction [W/m²]; 4 - Solar diffuse radiation [W/m²]
; 5 - Wind speed [m/s]; 6 - Wind direction [°]

; 7 - relative Humidity [%]"   annotation(Placement(transformation(extent={{0,65},{20,85}})));
  protected
    Real SecondOfYear "Second of year" annotation (
     HideResult=false,
     Dialog(
      group="Time and Date",
      tab="Results 1"));
  public
    Real HourOfDay "Time of day" annotation (
     Placement(
      transformation(extent={{175,10},{195,30}}),
      iconTransformation(
       origin={-50,-150},
       extent={{-10,-10},{10,10}},
       rotation=-90)),
     Dialog(
      group="Time and Date",
      tab="Results 1",
      visible=false));
    Real HourOfYear "Time of year" annotation (
     Placement(
      transformation(extent={{175,-10},{195,10}}),
      iconTransformation(
       origin={0,-150},
       extent={{-10,-10},{10,10}},
       rotation=270)),
     Dialog(
      group="Time and Date",
      tab="Results 1",
      visible=false));
    Real DayOfYear "Day of year" annotation (
     Placement(
      transformation(
       origin={115,-110},
       extent={{-10,-10},{10,10}},
       rotation=-90),
      iconTransformation(
       origin={-100,-150},
       extent={{-10,-10},{10,10}},
       rotation=-90)),
     Dialog(
      group="Time and Date",
      tab="Results 1",
      visible=false));
    Real DayOfWeek "Day of week (1: Mo, 2: Tue, 3: We, 4: Thu, 5: Fri, 6: Sa, 7: So)" annotation(Dialog(
     group="Time and Date",
     tab="Results 1",
     visible=false));
    Real DayOfMonth "Day of Month" annotation(Dialog(
     group="Time and Date",
     tab="Results 1",
     visible=false));
    Real MonthOfYear "Simulated Month (1: Jan, 2: Feb, 3: Mar, 4: Apr, 5: May, 6: Jun, 7: Jul, 8: Aug, 9: Sep, 10: Oct, 11: Nov, 12: Dec)" annotation(Dialog(
     group="Time and Date",
     tab="Results 1",
     visible=false));
    Real Year(start=1) "Simulated year" annotation(Dialog(
     group="Time and Date",
     tab="Results 1",
     visible=false));
  protected
    Boolean LeapYear "Simulated year is a leap year" annotation (
     Placement(
      transformation(
       origin={65,-110},
       extent={{-10,-10},{10,10}},
       rotation=-90),
      iconTransformation(
       origin={100,-150},
       extent={{-10,-10},{10,10}},
       rotation=-90)),
     Dialog(
      group="Time and Date",
      tab="Results 1"));
    Boolean SummerTime "Average location time is Summer time" annotation(Dialog(
     group="Time and Date",
     tab="Results 1"));
    Boolean Summer "Time period for Summer time" annotation(Dialog(
     group="Time and Date",
     tab="Results 1"));
    Real MonthDays "Days in actual Month" annotation(Dialog(
     group="Time and Date",
     tab="Results 1"));
    Integer Days[12] "Number of Days in a Month" annotation(Dialog(
     group="Time and Date",
     tab="Results 1"));
    Real RealHour "Real actually simulated hour" annotation(Dialog(
     group="Time and Date",
     tab="Results 1"));
    Real AverageHour "Averaged actually simulated hour" annotation(Dialog(
     group="Time and Date",
     tab="Results 1"));
    Integer DSTstartDay "First day of year with daylight saving time" annotation(Dialog(
     group="Time and Date",
     tab="Results 1"));
    Integer DSTendDay "Last day of year with daylight saving time" annotation(Dialog(
     group="Time and Date",
     tab="Results 1"));
    discrete Real aux[6] "Axiliary vector" annotation(Dialog(
     group="Time and Date",
     tab="Results 1"));
  public
    Modelica.Blocks.Interfaces.RealOutput UnixTime(quantity="Basics.Time") "Output of unix time" annotation (
     Placement(
      transformation(extent={{-110,40},{-90,60}}),
      iconTransformation(extent={{-140,-60},{-160,-40}})),
     Dialog(
      group="Time and Date",
      tab="Results 1",
      visible=false));
    Real RadiationVector[3] "3-dimensional radiation direction vector" annotation (
     Placement(
      transformation(extent={{175,-40},{195,-20}}),
      iconTransformation(extent={{140,-10},{160,10}})),
     Dialog(
      group="Solar Radiation",
      tab="Results 2",
      visible=false));
    Real RadiationDiffuse(
     start=0,
     quantity="Thermics.HeatFlowSurf",
     displayUnit="W/m²") "Solar diffuse radiation" annotation (
     Placement(
      transformation(extent={{175,-60},{195,-40}}),
      iconTransformation(extent={{140,90},{160,110}})),
     Dialog(
      group="Solar Radiation",
      tab="Results 2",
      visible=false));
    Real RadiationDirect(
     quantity="Thermics.HeatFlowSurf",
     displayUnit="W/m²") "Solar direct radiation in radiation direction" annotation (
     Placement(
      transformation(extent={{175,-85},{195,-65}}),
      iconTransformation(extent={{140,-110},{160,-90}})),
     Dialog(
      group="Solar Radiation",
      tab="Results 2",
      visible=false));
  protected
    Real ZenithAngle(
     quantity="Geometry.Angle",
     displayUnit="°") "Zenith angle of the sun position" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
    Real AzimuthAngle(
     quantity="Geometry.Angle",
     displayUnit="°") "Azimuth angle of the sun position" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
    Real SolarAltitude(
     quantity="Geometry.Angle",
     displayUnit="°") "Azimuth angle of the sun position" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
    Real RadiationVect[3] "Normal vector of radiation" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
    Real MeasurementAngle(
     quantity="Geometry.Angle",
     displayUnit="°") "Angle between measured radiation data vector and actual radiation vector" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
    Real solRadiation(
     start=0,
     quantity="Thermics.HeatFlowSurf",
     displayUnit="W/m²") "Global solar radiation" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
    Real DiffRad(
     quantity="Thermics.HeatFlowSurf",
     displayUnit="W/m²") "Diffuse solar radiation" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
    Real DirRad(
     start=0,
     quantity="Thermics.HeatFlowSurf",
     displayUnit="W/m²") "Direct solar radiation" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
    Real TimeEquation "Results of the global time equation" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
    Real J "Correction factor for the global time equation" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
    Real delta(
     quantity="Geometry.Angle",
     displayUnit="°") "Sun declination angle" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
    Real omega(
     quantity="Geometry.Angle",
     displayUnit="°") "Hour angle" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
    Real MeasurementVector[3] "Normal vector for measured radiation" annotation(Dialog(
     group="Solar Radiation",
     tab="Results 2"));
  public
    Real WindSpeed(
     quantity="Mechanics.Translation.Velocity",
     displayUnit="m/s") "Wind speed" annotation (
     Placement(
      transformation(extent={{175,60},{195,80}}),
      iconTransformation(
       origin={-100,150},
       extent={{-10,-10},{10,10}},
       rotation=90)),
     Dialog(
      group="Wind",
      tab="Results 2",
      visible=false));
  protected
    Real vWind(
     quantity="Mechanics.Translation.Velocity",
     displayUnit="m/s") "Wind speed" annotation(Dialog(
     group="Wind",
     tab="Results 2"));
  public
    Real WindVector[2] "Wind direction vector" annotation (
     Placement(
      transformation(extent={{175,35},{195,55}}),
      iconTransformation(
       origin={-50,150},
       extent={{-10,-10},{10,10}},
       rotation=90)),
     Dialog(
      group="Wind",
      tab="Results 2",
      visible=false));
  protected
    Real WindDirection(
     quantity="Geometry.Angle",
     displayUnit="°") "Wind direction" annotation(Dialog(
     group="Wind",
     tab="Results 2"));
  public
    Real TAmbient(
     quantity="Thermics.Temp",
     displayUnit="°C") "Ambient temperature" annotation (
     Placement(
      transformation(extent={{175,85},{195,105}}),
      iconTransformation(
       origin={100,150},
       extent={{-10,-10},{10,10}},
       rotation=-270)),
     Dialog(
      group="Ambient Temperature",
      tab="Results 2",
      visible=false));
  protected
    Real TAmb(
     quantity="Thermics.Temp",
     displayUnit="°C") "Ambient temperature" annotation(Dialog(
     group="Ambient Temperature",
     tab="Results 2"));
  public
    Real relHumidityAir(
     quantity="Basics.RelMagnitude",
     displayUnit="%") "Relative humidity" annotation(Dialog(
     group="Rel. Humidity",
     tab="Results 2",
     visible=false));
    parameter String Filename=CoSES_ProHMo.LibraryPath + "Data\\Weather\\Weather_2021_Munich_15min.txt" "Filename" annotation(Dialog(
     group="Weather Input Data",
     tab="Input Data"));
    parameter Real Altitude(
     quantity="Basics.Length",
     displayUnit="m")=519 "Altitude (Default: Munich)" annotation(Dialog(
     group="Weather Input Data",
     tab="Input Data"));
  protected
    parameter Real LongWaveCounterRadiation(
     quantity="Thermics.HeatFlowSurf",
     displayUnit="W/m²")=0 "Longwave counter radiation" annotation(Dialog(
     group="Weather Input Data",
     tab="Input Data"));
    parameter Real MeasureInclination=0 "Inclination angle for radiation data" annotation(Dialog(
     group="Weather Input Data",
     tab="Input Data"));
    parameter Real MeasureOrientation=0 "Orientation angle for radiation data" annotation(Dialog(
     group="Weather Input Data",
     tab="Input Data"));
  public
    parameter Boolean SmoothData=true "If enabled, input data is smoothed with PT1-behavior and 'SmoothTimeConstant'" annotation(Dialog(
     group="Data Smoothing",
     tab="Input Data"));
    parameter Real SmoothTimeConstant(
     quantity="Basics.Time",
     displayUnit="s")=10 if SmoothData "Time constant for PT1-smoothing of input data" annotation(Dialog(
     group="Data Smoothing",
     tab="Input Data"));
    parameter Real alpha(
     quantity="Geometry.Angle",
     displayUnit="°")=0.2020414784191361 "Longitude (Default: Munich)" annotation(Dialog(
     group="Location Data",
     tab="Time and Location"));
    parameter Real beta(
     quantity="Geometry.Angle",
     displayUnit="°")=0.8401518298395585 "Latidude (Default: Munich)" annotation(Dialog(
     group="Location Data",
     tab="Time and Location"));
  protected
    parameter Real StandardAlpha(
     quantity="Geometry.Angle",
     displayUnit="rad")=if (alpha/(15*pi/180)-floor(alpha/(15*pi/180))<0.5) then floor(alpha/(15*pi/180))*(15*pi/180) else ceil(alpha/(15*pi/180))*(15*pi/180) "Standard longitude of time zone" annotation(Dialog(
     group="Location Data",
     tab="Time and Location"));
  public
    parameter Boolean UseInitUnixTime=true "Use unix time for initialization if true, else use day/month/year init" annotation(Dialog(
     group="Initialization",
     tab="Time and Location"));
    parameter Real UnixTimeInit=1609455600 "Initial unix time" annotation(Dialog(
     group="Initialization",
     tab="Time and Location"));
    parameter Integer DayTimeInit=0 "Initial time of day (0-23)" annotation(Dialog(
     group="Initialization",
     tab="Time and Location"));
  protected
    parameter Real LeapYearNumber=floor((YearInit-1968-1)/4) "Number of leap years since 1970" annotation(Dialog(
     group="Initialization",
     tab="Time and Location"));
    parameter Integer LeapYearDay=if ((not (mod(floor(YearInit),4)>0)) or not (mod(floor(YearInit),400)>0)) then 1 else 0 "1 if leap year, else 0" annotation (
     HideResult=false,
     Dialog(
      group="Initialization",
      tab="Time and Location"));
    parameter Real DayInit=if MonthInit == 1 then MonthDayInit
    elseif MonthInit == 2 then MonthDayInit + 31
    elseif MonthInit == 3 then MonthDayInit + 59 + LeapYearDay
    elseif MonthInit == 4 then MonthDayInit + 90 + LeapYearDay
    elseif MonthInit == 5 then MonthDayInit + 120 + LeapYearDay
    elseif MonthInit == 6 then MonthDayInit + 151 + LeapYearDay
    elseif MonthInit == 7 then MonthDayInit + 181 + LeapYearDay
    elseif MonthInit == 8 then MonthDayInit + 212 + LeapYearDay
    elseif MonthInit == 9 then MonthDayInit + 243 + LeapYearDay
    elseif MonthInit == 10 then MonthDayInit + 273 + LeapYearDay
    elseif MonthInit == 11 then MonthDayInit + 304 + LeapYearDay
    else MonthDayInit + 334 + LeapYearDay "Initial Day of the year" annotation (
     HideResult=false,
     Dialog(
      group="Initialization",
      tab="Time and Location"));
  public
    parameter Integer MonthDayInit=1 "Initial day of Month (1-31)" annotation(Dialog(
     group="Initialization",
     tab="Time and Location"));
    parameter Integer MonthInit=1 "Initial Month of year (1-12)" annotation(Dialog(
     group="Initialization",
     tab="Time and Location"));
    parameter Integer YearInit=2021 "Initial year (>1970)" annotation(Dialog(
     group="Initialization",
     tab="Time and Location"));
    parameter Integer Timezone=1 "GMT + x (x = 1 for Central European Standard Time)" annotation(Dialog(
     group="Initialization",
     tab="Time and Location"));
  protected
    parameter Real UnixTimeInit_res=DayTimeInit * 3600 + (DayInit-1) * 86400 + (YearInit - 1970) * 31536000 + LeapYearNumber * 86400 - Timezone * 3600 "Initial Unix Timestamp" annotation (
     HideResult=false,
     Dialog(
      group="Initialization",
      tab="Time and Location"));
  public
    parameter Integer SummerTimeBegin=85 "Day in year when Summer time begins" annotation(Dialog(
     group="Daylight Saving Time",
     tab="Time and Location"));
    parameter Integer SummerTimeEnd=302 "Day in year when Summer time ends" annotation(Dialog(
     group="Daylight Saving Time",
     tab="Time and Location"));
    parameter Real cGround(
     quantity="Thermics.SpecHeatCapacity",
     displayUnit="kJ/(kg·K)")=1339 "Specific heat capacity of the ground" annotation(Dialog(
     group="Ground Properties",
     tab="Ground"));
    parameter Real lambdaGround(
     quantity="Thermics.SpecHeatCond",
     displayUnit="W/(m·K)")=1.45 "Heat conductivity of the ground" annotation(Dialog(
     group="Ground Properties",
     tab="Ground"));
    parameter Real rhoGround(
     quantity="Thermics.Density",
     displayUnit="kg/m³")=1800 "Density of ground" annotation(Dialog(
     group="Ground Properties",
     tab="Ground"));
    parameter Real GeoGradient(
     quantity="Thermics.TempDiff",
     displayUnit="K")=0.025 "Geothermal gradient" annotation(Dialog(
     group="Ground Properties",
     tab="Ground"));
    parameter Real alphaAirGround(
     quantity="Thermics.HeatTransmCoeff",
     displayUnit="W/(m²·K)")=1.8 "Heat transmission coefficient between ground and air" annotation(Dialog(
     group="Ground Properties",
     tab="Ground"));
    parameter Real TAverageAmbientAnnual(
     quantity="Thermics.Temp",
     displayUnit="°C")=282.90999999999997 "Average ambient temperature in a year" annotation(Dialog(
     group="Ambience",
     tab="Ground"));
    parameter Real TAmbientMax(
     quantity="Thermics.Temp",
     displayUnit="°C")=292.25 "Maximum monthly averaged ambient temperature during a year" annotation(Dialog(
     group="Ambience",
     tab="Ground"));
    parameter Integer MaxMonth=7 "Month with maximum average temperature" annotation(Dialog(
     group="Ambience",
     tab="Ground"));
    parameter Real cpAir(
     quantity="Thermics.SpecHeatCapacity",
     displayUnit="kJ/(kg·K)")=1004.9999999999999 "Specific heat capacity of air" annotation(Dialog(
     group="Air Specifications",
     tab="Air"));
    parameter Real rhoAir(
     quantity="Thermics.Density",
     displayUnit="kg/m³")=1.1839 "Density of air" annotation(Dialog(
     group="Air Specifications",
     tab="Air"));
  protected
    parameter Real CO2Concentration(
     quantity="Basics.RelMagnitude",
     displayUnit="ppm")=0.00039999999999999996 "CO2 Concentration" annotation(Dialog(
     group="Air Specifications",
     tab="Air"));
    parameter Real pAir0(
     quantity="Basics.Pressure",
     displayUnit="hPa")=101325 "Standard atmosphere air pressure at 0m height" annotation(Dialog(
     group="Boltzmann Barometric Equation",
     tab="Air"));
    parameter Real TAir0(
     quantity="Basics.Temp",
     displayUnit="°C")=288.14999999999998 "Standard atmosphere temperature at 0m height" annotation(Dialog(
     group="Boltzmann Barometric Equation",
     tab="Air"));
    parameter Real TemperatureGradient=0.0065 "Standard atmosphere temperature gradient [K/m]" annotation(Dialog(
     group="Boltzmann Barometric Equation",
     tab="Air"));
  public
    Modelica.Blocks.Tables.CombiTable1D Init(
     tableOnFile=true,
     tableName="T_StartEnd",
     fileName=Filename,
     columns={2,3}) "1 - Day Time Init
2 - Week Day Init
3 - Month Day Init
4 - Month Init
5 - Year Init"   annotation(Placement(transformation(extent={{0,0},{20,20}})));
    Modelica.Blocks.Sources.RealExpression realExpression1 "Set output signal to a time varying Real expression" annotation(Placement(transformation(extent={{-40,0},{-20,20}})));
  initial equation
    if (SmoothData) then
     assert(SmoothTimeConstant>1e-10,"smooth time constant equals 0");
    end if;

    vWind=WeatherData.y[4];

    WindDirection=angleRadian(WeatherData.y[6]);

    TAmb=WeatherData.y[1]+273.15;

    relHumidityAir = WeatherData.y[7]/100;

    if (ZenithAngle < angleRadian(90)) then
     solRadiation=DiffRad + DirRad;
     DiffRad=WeatherData.y[4];
     DirRad=WeatherData.y[3];
    else
     solRadiation=0;
     DiffRad=0;
     DirRad=0;
    end if;

    if UseInitUnixTime then
     assert((UnixTimeInit >= Init.y[1]) and (UnixTimeInit < Init.y[2]), "Initial time not within dataset");
     (HourOfDay, DayOfWeek, DayOfMonth, MonthOfYear, DayOfYear, Year, DSTstartDay, DSTendDay) = initTimeBaseUnix(UnixTimeInit);
    else
     assert((UnixTimeInit_res >= Init.y[1]) and (UnixTimeInit_res < Init.y[2]), "Initial time not within dataset");
     (HourOfDay, DayOfWeek, DayOfMonth, MonthOfYear, DayOfYear, Year, DSTstartDay, DSTendDay) = initTimeBaseUnix(UnixTimeInit_res);
    end if;

    assert((DayTimeInit>=0) and (DayTimeInit<24), "Time of day out of plausible limits");
    assert((MonthDayInit>=1) and (MonthDayInit<=31), "Day of Month out of plausible limits");
    assert((MonthInit>=1) and (MonthInit<=12), "Month out of plausible limits");
    assert(YearInit>=1970, "Year out of plausible limits (has to be past 1970)");

    if ((beta>0 and (DayOfYear<=DSTstartDay or (integer(DayOfYear)>DSTendDay or (integer(DayOfYear)==DSTendDay and HourOfDay>=3)))) or (beta<=0 and (DayOfYear<=DSTstartDay and (integer(DayOfYear)>DSTendDay or (integer(DayOfYear)==DSTendDay and HourOfDay>=3))))) then
     Summer=false;
    else
     Summer=true;
    end if;
  equation
    assert((UnixTime >= Init.y[1]) and (UnixTime <= Init.y[2]), "Environment time out of dataset", level = AssertionLevel.warning);

    // Time
    if ((not (mod(floor(Year),4)>0)) or not (mod(floor(Year),400)>0)) then
     LeapYear=true;
    else
     LeapYear=false;
    end if;

    Days={31, if LeapYear then 29 else 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    MonthDays=getVectorValue(Days, MonthOfYear);

    if ((beta>0 and (DayOfYear<=DSTstartDay or DayOfYear>DSTendDay)) or (beta<=0 and (DayOfYear<=DSTstartDay and DayOfYear>DSTendDay))) then
     SummerTime=false;
    else
     SummerTime=true;
    end if;

    when (SummerTime and HourOfDay>=2 and HourOfDay<=3 and not pre(Summer) and integer(DayOfYear)==DSTstartDay) then
     reinit(HourOfDay,3);
    elsewhen (not SummerTime and HourOfDay>=3 and HourOfDay<=4 and pre(Summer) and integer(DayOfYear)==DSTendDay) then
     reinit(HourOfDay,2);
    elsewhen (HourOfDay>=24) then
     reinit(HourOfDay,0);
    end when;

    when (LeapYear and (DayOfYear>366)) then
     reinit(DayOfYear,0);
     (aux[1], aux[2], aux[3], aux[4], aux[5], aux[6], DSTstartDay, DSTendDay) = initTimeBaseUnix(time);

    elsewhen (not (LeapYear) and (DayOfYear>365)) then
     reinit(DayOfYear,0);
     (aux[1], aux[2], aux[3], aux[4], aux[5], aux[6], DSTstartDay, DSTendDay) = initTimeBaseUnix(time);

    end when;

    when (SummerTime and (HourOfDay)>=2 and (HourOfDay)<=3 and not pre(Summer)) then
     Summer=true;
    elsewhen (not SummerTime and (HourOfDay)>=3 and (HourOfDay)<=4 and pre(Summer)) then
     Summer=false;
    end when;

    when (DayOfMonth >= getVectorValue(Days, MonthOfYear-0.1)+1) then
     reinit(DayOfMonth,1);
    elsewhen (SummerTime and HourOfDay>=2 and HourOfDay<=3 and not pre(Summer) and integer(DayOfYear)==DSTstartDay) then
     reinit(DayOfMonth,DayOfMonth+1/24);
    elsewhen (not SummerTime and HourOfDay>=3 and HourOfDay<=4 and pre(Summer) and integer(DayOfYear)==DSTendDay) then
     reinit(DayOfMonth,DayOfMonth-1/24);
    end when;

    when DayOfWeek>8 then
     reinit(DayOfWeek,1);
    elsewhen (SummerTime and HourOfDay>=2 and HourOfDay<=3 and not pre(Summer) and integer(DayOfYear)==DSTstartDay) then
     reinit(DayOfWeek,DayOfWeek+1/24);
    elsewhen (not SummerTime and HourOfDay>=3 and HourOfDay<=4 and pre(Summer) and integer(DayOfYear)==DSTendDay) then
     reinit(DayOfWeek,DayOfWeek-1/24);
    end when;

    when MonthOfYear>13 then
     reinit(MonthOfYear,1);
    elsewhen (SummerTime and HourOfDay>=2 and HourOfDay<=3 and not pre(Summer) and integer(DayOfYear)==DSTstartDay) then
     reinit(MonthOfYear,MonthOfYear+1/(24*MonthDays));
    elsewhen (not SummerTime and HourOfDay>=3 and HourOfDay<=4 and pre(Summer) and integer(DayOfYear)==DSTendDay) then
     reinit(MonthOfYear,MonthOfYear-1/(24*MonthDays));
    end when;

    if UseInitUnixTime then
     UnixTime = UnixTimeInit + time;
    else
     UnixTime = UnixTimeInit_res + time;
    end if;

    der(HourOfDay)=der(time)/3600;
    der(DayOfMonth)=der(time)/(3600*24);
    der(MonthOfYear)=der(time)/(3600*24*MonthDays);
    der(DayOfWeek)=der(time)/(3600*24);
    der(Year)=der(time)/(3600*24*sumOfDays(Days,1,12));
    der(DayOfYear)=der(time)/(3600*24);
    HourOfYear=DayOfYear*24;

    if noEvent(LeapYear) then
     J=angleRadian(360)*DayOfYear/365;
    else
     J=angleRadian(360)*DayOfYear/366;
    end if;

    TimeEquation=0.0066+7.3525*cos(J+angleRadian(85.9))+9.9359*cos(2*J+angleRadian(108.9))+0.3387*cos(3*J+angleRadian(105.2));

    if pre(Summer) then
     if noEvent((HourOfDay-1-(1/angleRadian(15))*(StandardAlpha-alpha))<0) then
      AverageHour=24+HourOfDay-1-(1/angleRadian(15))*(StandardAlpha-alpha);
     elseif noEvent((((HourOfDay-1-(1/angleRadian(15))*(StandardAlpha-alpha))>=0)) and ((HourOfDay-1-(1/angleRadian(15))*(StandardAlpha-alpha)) < 24)) then
      AverageHour=HourOfDay-1-(1/angleRadian(15))*(StandardAlpha-alpha);
     else
      AverageHour=HourOfDay-1-(1/angleRadian(15))*(StandardAlpha-alpha)-24;
     end if;
    else
     if noEvent((HourOfDay-(1/angleRadian(15))*(StandardAlpha-alpha))<0) then
      AverageHour=24+HourOfDay-(1/angleRadian(15))*(StandardAlpha-alpha);
     elseif noEvent((((HourOfDay-(1/angleRadian(15))*(StandardAlpha-alpha))>=0)) and ((HourOfDay-1-(1/angleRadian(15))*(StandardAlpha-alpha)) < 24)) then
      AverageHour=HourOfDay-(1/angleRadian(15))*(StandardAlpha-alpha);
     else
      AverageHour=HourOfDay-(1/angleRadian(15))*(StandardAlpha-alpha)-24;
     end if;
    end if;

    if noEvent(((TimeEquation/60)+AverageHour) < 0) then
     RealHour=24+((TimeEquation/60)+AverageHour);
    elseif noEvent((((TimeEquation/60)+AverageHour) >= 0) and (((TimeEquation/60)+AverageHour) < 24)) then
     RealHour=(TimeEquation/60)+AverageHour;
    else
     RealHour=(TimeEquation/60)+AverageHour-24;
    end if;

    omega=(12-RealHour)*angleRadian(15);

    delta=angleRadian(0.3948-23.2559*cos(J+angleRadian(9.1))-0.3915*cos(2*J+angleRadian(5.4))-0.1764*cos(3*J+angleRadian(26)));
    ZenithAngle=acos(sin(delta)*sin(beta)+cos(delta)*cos(beta)*cos(omega));
    SolarAltitude=pi/2-ZenithAngle;
    AzimuthAngle=getAzimuth(RealHour,SolarAltitude,beta,delta);

    RadiationVect=normalVector(ZenithAngle,AzimuthAngle);
    for i in 1:3 loop
     RadiationVector[i]=RadiationVect[i];
    end for;

    MeasurementVector=normalVector(angleRadian(MeasureInclination), angleRadian(MeasureOrientation));
    MeasurementAngle=diffAngle(RadiationVect,MeasurementVector);

    when (ZenithAngle >= angleRadian(90)) then
     if not SmoothData then
      reinit(solRadiation,0);
      reinit(DiffRad,0);
      reinit(DirRad,0);
     end if;
    elsewhen (ZenithAngle < angleRadian(90)) then
     if not SmoothData then
      reinit(solRadiation,DiffRad + DirRad);
      reinit(DiffRad,WeatherData.y[4]);
      reinit(DirRad,WeatherData.y[3]);
     end if;
    end when;

    if (ZenithAngle < angleRadian(90)) then
     if noEvent(SmoothData) then
      SmoothTimeConstant*der(solRadiation)+solRadiation=DiffRad + DirRad;
      SmoothTimeConstant*der(DiffRad)+DiffRad=WeatherData.y[4];
      SmoothTimeConstant*der(DirRad)+DirRad=WeatherData.y[3];
     else
      der(solRadiation)=der(DiffRad + DirRad);
      der(DiffRad)=der(WeatherData.y[4]);
      der(DirRad)=der(WeatherData.y[3]);
     end if;
    else
     if noEvent(SmoothData) then
      SmoothTimeConstant*der(solRadiation)+solRadiation=0;
      SmoothTimeConstant*der(DiffRad)+DiffRad=0;
      SmoothTimeConstant*der(DirRad)+DirRad=0;
     else
      der(solRadiation)=0;
      der(DiffRad)=0;
      der(DirRad)=0;
     end if;
    end if;

    if noEvent(SmoothData) then
     SmoothTimeConstant*der(vWind)+vWind=WeatherData.y[5];

     SmoothTimeConstant*der(WindDirection)+WindDirection=angleRadian(WeatherData.y[6]);

     SmoothTimeConstant*der(TAmb)+TAmb=WeatherData.y[1]+273.15;

     SmoothTimeConstant*der(relHumidityAir)+relHumidityAir=WeatherData.y[7]/100;
    else
     if (HourOfDay<23.5 and HourOfDay>0.5) then
      der(vWind)=der(WeatherData.y[5]);

      der(WindDirection)=der(angleRadian(WeatherData.y[6]));

      der(TAmb)=der(WeatherData.y[1]+273.15);

      der(relHumidityAir) = der(WeatherData.y[7]/100);
     else
      der(vWind)+vWind=WeatherData.y[5];

      der(WindDirection)+WindDirection=angleRadian(WeatherData.y[6]);

      der(TAmb)+TAmb=WeatherData.y[1]+273.15;

      der(relHumidityAir)+relHumidityAir = WeatherData.y[7]/100;
     end if;
    end if;

    WindVector=normalVector2D(WindDirection);
    WindSpeed=vWind;
    TAmbient=TAmb;
    RadiationDiffuse=DiffRad;
    RadiationDirect=DirRad;

    // Environment Port
    EnvironmentConditions.DayOfMonth=DayOfMonth;
    EnvironmentConditions.HourOfDay=HourOfDay;
    EnvironmentConditions.MonthOfYear=MonthOfYear;
    EnvironmentConditions.Year=Year;
    EnvironmentConditions.DayOfWeek=DayOfWeek;
    EnvironmentConditions.DayOfYear=DayOfYear;
    EnvironmentConditions.HourOfYear=HourOfYear;
    EnvironmentConditions.LeapYear=LeapYear;
    EnvironmentConditions.RadiationDiffuse=DiffRad;
    EnvironmentConditions.RadiationVector=normalVector(ZenithAngle,AzimuthAngle);
    EnvironmentConditions.WindSpeed=vWind;
    EnvironmentConditions.WindVector=normalVector2D(WindDirection);
    EnvironmentConditions.TAmbient=TAmb;
    EnvironmentConditions.TAmbientAverageAct=WeatherData.y[2]+273.15;
    EnvironmentConditions.cGround=cGround;
    EnvironmentConditions.GeoGradient=GeoGradient;
    EnvironmentConditions.lambdaGround=lambdaGround;
    EnvironmentConditions.MaxMonth=MaxMonth;
    EnvironmentConditions.rhoGround=rhoGround;
    EnvironmentConditions.TAmbientMax=TAmbientMax;
    EnvironmentConditions.TAverageAmbientAnnual=TAverageAmbientAnnual;
    EnvironmentConditions.rhoAir=rhoAir;
    EnvironmentConditions.cpAir=cpAir;
    EnvironmentConditions.alphaAirGround=alphaAirGround;
    EnvironmentConditions.RadiationDirect=DirRad;
    EnvironmentConditions.pAir=pAir0*(1-(TemperatureGradient*Altitude/TAir0))^5.255;
    EnvironmentConditions.relHumidityAir=relHumidityAir;
    EnvironmentConditions.LongWaveCounterRadiation=LongWaveCounterRadiation;
    EnvironmentConditions.CO2Concentration=CO2Concentration;
    connect(TimeBase.y,WeatherData.u[2]) annotation(Line(
     points={{-39,75},{-34,75},{-7,75},{-2,75}},
     color={0,0,127},
     thickness=0.0625));
    connect(TimeBase.y,WeatherData.u[3]) annotation(Line(
     points={{-39,75},{-34,75},{-7,75},{-2,75}},
     color={0,0,127},
     thickness=0.0625));
    connect(TimeBase.y,WeatherData.u[4]) annotation(Line(
     points={{-39,75},{-34,75},{-7,75},{-2,75}},
     color={0,0,127},
     thickness=0.0625));
    connect(TimeBase.y,WeatherData.u[5]) annotation(Line(
     points={{-39,75},{-34,75},{-7,75},{-2,75}},
     color={0,0,127},
     thickness=0.0625));
    connect(realExpression1.y,Init.u[1]) annotation (
     Line(
      points={{-19,10},{-14,10},{-7,10},{-2,10}},
      color={0,0,127},
      thickness=0.0625),
     __esi_AutoRoute=false);
    connect(realExpression1.y,Init.u[2]) annotation (
     Line(
      points={{-19,10},{-14,10},{-7,10},{-2,10}},
      color={0,0,127},
      thickness=0.0625),
     __esi_AutoRoute=false);
    connect(TimeBase.y,WeatherData.u[6]) annotation(Line(
     points={{-39,75},{-34,75},{-7,75},{-2,75}},
     color={0,0,127},
     thickness=0.0625));
    connect(TimeBase.y,WeatherData.u[1]) annotation(Line(
     points={{-39,75},{-34,75},{-7,75},{-2,75}},
     color={0,0,127},
     thickness=0.015625));
    connect(TimeBase.y,WeatherData.u[7]) annotation(Line(
     points={{-39,75},{-34,75},{-7,75},{-2,75}},
     color={0,0,127},
     thickness=0.0625));
   annotation (
    Icon(
     coordinateSystem(extent={{-150,-150},{150,150}}),
     graphics={
         Bitmap(
          imageSource="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
uv8DpclLs2N3+28AAAAASUVORK5CYII=",
          extent={{-150,-150},{150,150}})}),
    Documentation(info="<HTML><HEAD>
<META http-equiv=\"Content-Type\" content=\"text/html; charset=iso-8859-1\"> 
<TITLE>Environment with user defined weather profiles</TITLE> 
<STYLE type=\"text/css\">
table>tbody>tr:hover,th{background-color:#edf1f3}html{font-size:90%}body,table{font-size:1rem}body{font-family:'Open Sans',Arial,sans-serif;color:#465e70}h1,h2{margin:1em 0 .6em;font-weight:600}p{margin-top:.3em;margin-bottom:.2em}table{border-collapse:collapse;margin:1em 0;width:100%;}tr{border:1px solid #beccd3}td,th{padding:.3em 1em; border-width: 1px; border-style: solid; border-color: rgb(190, 204, 211);}td{word-break:break-word}tr:nth-child(even){background-color:#fafafa}th{font-family:'Open Sans Semibold',Arial,sans-serif;font-weight:700;text-align:left;word-break:keep-all}.lib-wrap{width:70%}@media screen and (max-width:800px){.lib-wrap{width:100%}}td p{margin:.2em}
</STYLE>
 
<META name=\"GENERATOR\" content=\"MSHTML 11.00.10570.1001\"></HEAD> 
<BODY>
<H1>Environment with user defined weather profiles</H1>
<P>Based on GreenCity's 'Environment2'<BR>Changes:<BR></P>
<UL>
  <LI>Individual time series for weather data can be included</LI></UL>
<HR>

<DIV class=\"lib-wrap\">
<TABLE class=\"type\">
  <TBODY>
  <TR>
    <TH>Symbol:</TH>
    <TD colspan=\"3\"><IMG width=\"203\" height=\"98\" src=\"data:image/png;base64,
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\"></TD></TR>
  <TR>
    <TH>Identifier:</TH>
    <TD colspan=\"3\">CoSES_Models.Environment.Environment</TD></TR>
  <TR>
    <TH>Version:</TH>
    <TD colspan=\"3\">1.0</TD></TR>
  <TR>
    <TH>File:</TH>
    <TD colspan=\"3\">Environment.mo</TD></TR>
  <TR>
    <TH>Connectors:</TH>
    <TD>Enviromnent Conditions Connection</TD>
    <TD>EnvironmentConditions</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>Parameters:</TH>
    <TD>Set output signal to a time varying Real expression</TD>
    <TD>TimeBase</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>0 - time[s] ; 1 - Ambient temperature [°C] ; 2 - 7 day average 
      temperature [°C]; 3 - Solar direct radiation in radiation direction 
      [W/m²]; 4 - Solar diffuse radiation [W/m²] ; 5 - Wind speed [m/s]; 6 - 
      Wind direction [°] ; 7 - relative Humidity [%]</TD>
    <TD>WeatherData</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>If true: Data from reference day/week</TD>
    <TD>ReferenceDay</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>File name for weather data</TD>
    <TD>InputFile</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Structure: time [s], Ambient temperature [°C], Solar direct radiation 
      in radiation direction [W/m²], Solar diffuse radiation [W/m²], Wind speed 
      [m/s], Wind direction [°]</TD>
    <TD>TableName</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Altitude (Default: Munich)</TD>
    <TD>Altitude</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>If enabled, input data is smoothed with PT1-behavior and 
      'SmoothTimeConstant'</TD>
    <TD>SmoothData</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Time constant for PT1-smoothing of input data</TD>
    <TD>SmoothTimeConstant</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Longitude (Default: Munich)</TD>
    <TD>alpha</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Latidude (Default: Munich)</TD>
    <TD>beta</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Use default data for time initialization</TD>
    <TD>DefaultInit</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Time base is calculated via unix time stamp input in SimulationX's 
      Property Dialog</TD>
    <TD>useUnixTimeStamp</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Initial time of day</TD>
    <TD>DayTimeInit</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Initial day of week</TD>
    <TD>WeekDayInit</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Initial day of Month</TD>
    <TD>MonthDayInit</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Initial Month of year</TD>
    <TD>MonthInit</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Initial year</TD>
    <TD>YearInit</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>If enabled, Summer time is used for calculation of actual time</TD>
    <TD>useSummerTime</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Day in year when Summer time begins</TD>
    <TD>SummerTimeBegin</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Day in year when Summer time ends</TD>
    <TD>SummerTimeEnd</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Specific heat capacity of the ground</TD>
    <TD>cGround</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Heat conductivity of the ground</TD>
    <TD>lambdaGround</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Density of ground</TD>
    <TD>rhoGround</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Geothermal gradient</TD>
    <TD>GeoGradient</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Heat transmission coefficient between ground and air</TD>
    <TD>alphaAirGround</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Average ambient temperature in a year</TD>
    <TD>TAverageAmbientAnnual</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Maximum monthly averaged ambient temperature during a year</TD>
    <TD>TAmbientMax</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Month with maximum average temperature</TD>
    <TD>MaxMonth</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Specific heat capacity of air</TD>
    <TD>cpAir</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Density of air</TD>
    <TD>rhoAir</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>1 - Day Time Init 2 - Week Day Init 3 - Month Day Init 4 - Month Init
       5 - Year Init</TD>
    <TD>Init</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Set output signal to a time varying Real expression</TD>
    <TD>realExpression1</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>Results:</TH>
    <TD>Time of day</TD>
    <TD>HourOfDay</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Time of year</TD>
    <TD>HourOfYear</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Day of year</TD>
    <TD>DayOfYear</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Day of week (1: Mo, 2: Tue, 3: We, 4: Thu, 5: Fri, 6: Sa, 7: So)</TD>
    <TD>DayOfWeek</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Day of Month</TD>
    <TD>DayOfMonth</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Simulated Month (1: Jan, 2: Feb, 3: Mar, 4: Apr, 5: May, 6: Jun, 7: 
      Jul, 8: Aug, 9: Sep, 10: Oct, 11: Nov, 12: Dec)</TD>
    <TD>MonthOfYear</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Simulated year</TD>
    <TD>Year</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>3-dimensional radiation direction vector</TD>
    <TD>RadiationVector</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Solar direct radiation in radiation direction</TD>
    <TD>RadiationDirect</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Solar diffuse radiation</TD>
    <TD>RadiationDiffuse</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Wind speed</TD>
    <TD>WindSpeed</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Wind direction vector</TD>
    <TD>WindVector</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Ambient temperature</TD>
    <TD>TAmbient</TD>
    <TD>&nbsp;</TD></TR>
  <TR>
    <TH>&nbsp;</TH>
    <TD>Relative humidity</TD>
    <TD>relHumidityAir</TD>
    <TD>&nbsp;</TD></TR></TBODY></TABLE></DIV>
<H2>Description:</H2>
<P>&nbsp;From Green City's 'Environment2':</P>
<P><BR></P>
<P>The <I>Environment</I> model has four major  functions within the GreenCity 
library:</P>
<P><BR></P>
<UL>
  <LI value=\"1\">Import of measurement data  for ambient temperature, solar 
  radiation, wind speed and wind  direction</LI>
  <LI value=\"2\">Definition of time base,  date, time of day, season and initial  
  parameters related to the simulated time period </LI>
  <LI value=\"3\">Calculation of the solar  inclination angle dependent on the 
  simulated season and correction of solar  radiation measurement data regarding 
  measurement angle</LI>
  <LI value=\"4\">Definition of ground and air  characteristics</LI></UL>
<P><BR></P>
<P>The parameter <CODE>InputData</CODE> defines the             weather data 
record which is used to specify the ambient conditions data for the              
simulation. Note that the weather input selection changes depending            
on the parameter <CODE>TRY</CODE>. If this parameter is set to <I>true</I>, a 
city-specific <I>Test Reference Year</I> (TRY) can be selected.             This 
provides the simulation with weather data of a complete year.            If set 
to <I>false</I>, the input options change to weather data sets of reference days 
(i.e. type days).             The following abbreviations are used for reference 
days:</P>
<P><BR></P>
<UL>
  <LI value=\"1\">First letter:                         </LI>
  <UL>
    <LI value=\"1\">W – Winter                             </LI>
    <LI value=\"2\">T – Transit                             </LI>
    <LI value=\"3\">S – Summer                             </LI></UL>
  <LI value=\"2\">Second letter:                         </LI>
  <UL>
    <LI value=\"1\">s – Sunny (&lt;3/8 clouds)                             </LI>
    <LI value=\"2\">o – Overcast (&gt;3/8 clouds)                             
</LI>
    <LI value=\"3\">r – High radiation (global direct and diffuse radiation higher 
    than monthly average)                             </LI>
    <LI value=\"4\">n – Low radiation                             </LI></UL>
  <LI value=\"3\">Third letter:                         </LI>
  <UL>
    <LI value=\"1\">w – Windy                             </LI>
    <LI value=\"2\">c – Calm                             </LI></UL></UL>
<P><BR></P>
<P>Note that each reference  day has an average day and a real day that can be 
selected.                 The average day includes the mean weather conditions 
of  all the days of this day type. The real day option on the other hand  uses   
              the day with the lowest weighted standard deviation from the 
average day. The simulated time always starts at the described date and time 
(usually 12 am), e.g. 11/15 refers to November 15th.  Initialization of the 
simulated time can be modified in parameter  dialogue <I>Time and Location</I> 
with the following parameters:</P>
<P><BR></P>
<UL>
  <LI value=\"1\"> DayTimeInit: First hour of the day (always integer: 0 .. 00:00 
  to 23 .. 23:00)                         </LI>
  <LI value=\"2\"> WeekDayInit: First day of the week (always integer: 1: monday, 
  7: sunday)                         </LI>
  <LI value=\"3\"> MonthDayInit: First day of the month (1.. 1st to (max) 31.. 
  31st)                         </LI>
  <LI value=\"4\"> MonthInit: First month of the year (1 .. january to 12 .. 
  december)                         </LI>
  <LI value=\"5\"> YearInit: First year                         </LI></UL>
<P><BR></P>
<P>The real-world location  to be simulated is specified through longitude and 
latitude. It is  pre-parametrized for the coordinates of the chosen city:</P>
<P><BR></P>
<UL>
  <LI value=\"1\">Longitude: -180° (West) to 180° (East)</LI>
  <LI value=\"2\">Latitude: -90° (South) to 90° (North)</LI></UL>
<P><BR></P>
<P>The current season and  time of day             are simulated depending on 
the simulated time period and  initial values for the time of             day, 
the day of month, the month and the year. This allows for calculating varying 
time-dependent conditions, such as leap             years and daylight saving 
time. If enabled, the daylight  saving time simulation depends on the            
 simulated day of the year and the chosen hemisphere  (latitude).</P>
<P><BR></P>
<P>Input data sets can include noisy measurement data with high gradients. This 
can cause numerical             issues. To prevent this, the simulation model 
includes a smoothing option which can be activated with the parameter 
<CODE>SmoothData</CODE>.            This method uses a defined time constant to 
smooth the input data through PT1 behavior.</P>
<P><BR></P>
<P>To calculate the current  solar inclination angle, the real local time is 
required. The time is  determined with the             location's longitude and 
the simulated day of the year. The  influences of the eccentricity of the 
earth's orbit is taken into  account             to ensure accurate 
calculations.</P>
<P><BR></P>
<P>With the calculated real local time, the current hour angle             (&#982;) 
is calculated as follows:</P>
<P><BR></P>
<UL>
  <LI value=\"1\">&#982;=(12.00 - RealLocalTime) * 15°/h</LI></UL>
<P><BR></P>
<P>Furthermore, the maximum solar altitude (&#948;) depends on the actual day of year 
and is determined as follows:</P>
<P><BR></P>
<UL>
  <LI value=\"1\">&#948;=0.3948 -  23.2559 * cos(DayOfYear/365 + 9.1°) - 0.3915 * cos(2 
  * DayOfYear/365 +  5.4°) - 0.1764 * cos(3 * DayOfYear/365 + 26.0°)</LI></UL>
<P><IMG align=\"baseline\" alt=\"\" src=\"data:image/png;base64,
iVBORw0KGgoAAAANSUhEUgAAAz4AAAFHCAIAAADvGw6vAAAgAElEQVR4nOzde1xTZ7Yw/rVzByRBuSiKoBatVi1VpxarrQptrbbBXrQjdjy2/QlCT6t2oPNr54Rf375w2vmdwY46cyoCn1aPM8DU9kxf0mppB6yjrdQeVAatinghRZFwSwKEJDvJfv94YBuSEALknvX9zKez2dns/QBqFut51noohmEAIYQQQgj5A463B4AQQgghhJyFoRtCCCGEkN/A0A0hhBBCyG9g6IYQQggh5DcwdEMIIYQQ8hsYuiGEEEII+Q0M3RBCCCGE/AaGbgghhBBCfgNDN4QQQgghv4GhG0IIobsUCkVNTY1arfb2QBBC9mHohhBC6K4jR46kpqaWlpZ6eyAIIfswdEMIIQ9pbGxMS0ujKCo7O1upVNq9pra2lqKokpISD48NIeQvMHRDCCEPaWlpkcvlAFBUVPTHP/7R7jVarRYANBqNW0dSU1NTU1Pj1kcghNwEQzeEEPIoqVQKAAUFBfX19V4ZgEKhSE1NTU1NVSgUXhkAQmg8MHRDCCGPWrlyZWFhIQAUFRV5eywIIf+DoRtCCHlUbm7uxo0bAaCoqGhss5YKhWJUCTOlUjmeBNtoH4cQcisM3RBCyNPi4+NJ4m3Pnj3Of1Z9fX1eXh5FUQkJCQkJCaTcwe6sa0lJCUVRtbW1ALB79+7JkycnJCSkpaWRGoiEhARyGbkJRVFpaWl2n1hTU5OWlkYel5aWhsvjEPIFGLohhJAXkMSbXC6vqKhw5vqKiooHHnigoKAAAGQymUwmA4CioqIHHnjA9g6kyqG9vT0vLy83N5ecJBUSdtm+pFKp8vLyUlNT2Zfkcrnlhwghb8HQDSGEvCA+Pr64uBgA0tPTR+x/W1NTk56eDgDl5eUMw+Tn5+fn5zMMU15eTu5gNx+WlpZWUFBQXl6uUqlUKtX58+eTk5MZhmlubiYXNDc3M4OsPregoKCgoKCyspK8qlKpSLCYlpaG3XoR8i4M3RBCyDteeOEFcvDJJ584vpLMqxYXF2/atMny/KZNm0hENdzEK/kUiUQikUiSkpJGNbzz58+TYlgAkEgkr7/+Ojm+dOnSqO6DEHItDN0QQsg7JBJJZWUlAGRmZjqoA6ivryfTlOvXr7d9dcOGDQAgl8tt7yCVSjMyMsY2NplMZhXqxcTEkEiOdJ5DCHkLhm4IIeQ1UqmUxENHjhwZ7prOzk5yEBMTY/tqbGwsObh9+7bVSytXrhzzwCIiImxPjueGCCFXwdANIYS8adeuXQCQm5s7XIfec+fOAUBWVpbdV9l4DpNhCAUJDN0QQsibUlJSSFiGHXoRQs7A0A0hhLyMDd3sJt6mTZsGwwd27Db2kZGRbhsgQsiHYOiGEEJelpSUxG6N1dfXZ/XqjBkzyAEbpVlqbW0lB+yiN4RQYMPQDSGEvI/dGqukpMTqpVmzZpEDu83bPv30UwDIysqyW8TgmE6nG/VAEULehqEbQgh5H9uh13a7gpiYGJKTS09PJ3tbsUpKSsj+Cm+88caonkUOTpw4QQ6wyy5CfgRDN4QQ8glsh15bOTk5pPXusmXL0tLSdu/evXv3boqiMjMzAaC6unrOnDmjehYJEzMzM7Ozs/Py8uy2AkEI+SaetweAEELBIjQ0FADYLQqsSCSS8+fPP/DAAwAgFoutXs3Pz58/f35ZWZlcLmczczKZbMuWLcPFbbY3YWVkZGg0mtzcXFL9YDskB5+LEPIuynbrOoQQQj6L3TWBbG81nlup1Wq1Wi0SicawTg4h5C0YuiGEEEII+Q1c64YQCmR37tzx9hAQQsiVMHRDCAUsnU63bNkyjN4QQoEEQzeEUMB69913b968mZ2d7e2BIISQy+BaN4RQYLp58+a8efNI19njx4+vWrXK2yNCCCEXwKwbQigwvfHGG+xuAaPqWIsQQr4MQzeEUAD66quvPv/8c/bD8+fP79mzx4vjQQghV8EJU4SCGunsBQBeae7FPp3dmslWRUXFiRMnbt26JZVK169f78wgSXXC+fPnLU9OmTLl3LlzU6ZMGf+wEULIizDrhlAwUqvVFRUVaWlpERERCQkJCQkJkydPpiiqpKSE7fjqVrW1tdnZ2ezTKYravXu3Uqm0uiwvLy89PX3x4sWbN2/OzMzctm2bM7ttFhUVWcVtAHDnzp13333XZV8AQgh5CWbdEAo69fX1ZLclgmyOSXYxJyorK4fbrMklKioq0tPTyXFhYeGJEyfIzk5SqbS0tJTNq5FxlpeXb9q0CQBqa2uXLVtWXV2dkpLi4OZ37tyZN2+eSqWy++q5c+csv3aEEPI7mHVDKLiwcZtMJrty5QrDMPn5+fn5+QzDnD9/PisrCwDS0tLYXTJdrqamhsRt1dXVDMPk5ORUVlY2NzdLpVK5XP7OO++wV3Z2dgLAww8/TD6cOnUqAJw7d87x/d99910St82YMYM9+cwzz5ADbBSCEPJ7DEIoaLC5KJlMNtw1JAkHAM3Nze4YA8nnFRcXW51npzjPnz9veYa9srq6GgBOnz7t4OaWgd3f/vY39vjSpUsikYgcl5eXu+PrQgghz8CsG0JB5NixY+QgNzd3uGsyMjLIwffff+/yATQ2NpJ83vr1661eSkpKIgdnzpxhz8hksszMzLy8vN27d6empspksuTkZAf3Z5NqTz75JJtpA4C5c+eShCIMbRqCEEJ+B0M3hIJIWVkZAJSXl0skkuGuiY+PJ4k3crFrXblyBQCkUqndQtHCwkIAOHv2LHsmPz+/srIyIiKCDDs/P9/BzSsqKmprawFAJBK9//77Vq++//77pLwU6xUQQn6N5+0BIIQ8RKFQkIzXvHnzHF+5evXqgoICuVyuVCpHbMbBNvhwgO390djYCAArV660e9mcOXMAoKioaP/+/exJ5wsmqqqqyEFWVpZtLQKJ515++WUA+Oqrr2xjO4QQ8gsYuiEULG7fvk0OJk6c6PjKuLg4cuDMxGJdXV1qaqrja9iy0OEKP4mwsLARH+fA+vXrDx48mJycbFnrYOmll16qqqqqqKiwna4NHm7qpYcQ8hgM3RAKFlqtlhzU19c3NTU5uJLtr3b79m0Hb/BEXFwcmeh0fA05sGxB4oBCoRjxubaeeeYZZqRuR+Xl5eXl5aO9c2Cora09dOhQUVERe6awsHDLli1WkVleXl5BQUFxcfHKlSvT09Plcvnhw4cdzLAjhDwMQzeEgk5aWpqTV7LRngNz5szJyckZ34iQ29ntpZebm3vixAmrXnoFBQVsL70ZM2YsW7asrq7OcS89hJAnYeiGUNCprKx0PDWpVCrJ2zybLUN+zbKXHgnCcnJyFArFa6+9RnrpsYsLh+ulh6EbQr4DQzeEgk5SUpLj6UhSTAAAbC80V8nKyrKcsBvOGGZLkQN79uwBgOLiYssILD4+Pj8/Xy6XFxUVZWVlkeYskZGRAFBVVUV6xJCJ9eXLl3tn3AghezB0QyhYsCm07u5ux7FRS0sLOXAmhGJbtTmwceNGcqvExESnxhoElEqlTqfzQJDqZC89csz20lMoFBEREbm5uSP20kMIeRiGbggFi8mTJ5ODS5cuse/Zdh0/fhwA2B62cGw7ta7YznWZR5kDa1taWhw0+CUWLVpEYhSxWAwAubm5dpfHkb0Q2O0cAlVNTc2ePXvYeLewsHDbtm0SiUSpVJKfkUqlcmFZwIi99HJzc6166S1dupRkXtlFbwgh34GhG0LBQiKRyGSygoKCsrIyB+/HCoWC1IFu3Lhx4NTaAwxzwPKapr3LZ+/6/uH7ZgPAggULyBZVDixYsIAcLFy4kBzY7Rh34sQJAFi6dKnTX5P/IfWbACCVSleuXJmbm0tqBSorK9lWLCPGbb7TSw8h5AXe3okLIeQ57D6htluIskjSSyqVDn+bo5kA8PCeq6MfANvXzXYj0dOnT5OX2traRn9j+3ztHzo2xrXcibW4uJh8Q8irDraXtb2PA9XV1eRi8gMtLCx0fCuXfIEIIQ/ArBtCQSQpKYlMkGVmZmo0GnYJGlFfX19UVETKCBy0aju2fV0xQKZs5xiWrUkkEjKA9PT0GTNmsIuoGhs
b33vvPfLcAG4Ay5YLWK4ey8jIkMvlZWVlJNc1f/78Ee/jO730EEJe4O3YESHkaVbv+oWFhZbLy6RS6fnz54f95Kt7HgaAzKNjfrpKpWLn46RSaWFhIbuozpmE06j41D90zc3NZDBXrlyxeqmyshIGFxc6+uaPCftTtvsqm3Vrbm527XMRQm7iE/+iIYQ87Pz587bVAFKptLy8XKVSOfjEsc+VWlCpVGSW0JLtFOr4+VTo5mBqklQSEI6//2OAoRtCAQYnTBEKRklJSUlJSfn5+QqFgpwRiUQjz1Qe276uGB7ec2gsc6UWJBJJRkZGRkYG+/Qgn6pj++dlZWW5fMsp7KWHUIDB0A2hoDaaN+ymvQVjXuQ2/qcHhXXr1jlzGfbSQyiYYeiGEHJK096tu76Hh/e8udbbIwk83d3d5MBxvz0W9tJDKJhh6IYQcsax3+/63gVzpUGM7DEF9nraXbp0aVS3wl56CAUzDN0QQiMbT0MQRMyYMYMc/PDDD1Y9b8vKysjBiHuUETExMc7vBz9v3jxyUFNTY9WKuba2lky8PvTQQ07eDSHkdRxvDwAh5PMGFrkdPYBzpeNAetoBQFpaGlufAQB5eXlyuZx0Bjlz5oz7npuenl5bW8ueD5JeeggFHoqxqJ9HCCFbZNsre69kHmV8OpyjKIoc+Mg/dEqlctu2bSTRJZPJIiIiTpw4IZfLCwsLH3vssQceeICc7+jo2Lp1a3Jy8tDv/di/22q1esuWLeS5ZAOupqYmUnYqk8ny8/Nd9PUhhDwBQzeEUMDytdANAJRK5eHDhy2LDCorK8n8aUVFRXp6Ojl5/vz5pKSkY9updRf2XP1uZyIANO1dvhUOfTfGOWu1Wv3JJ59kZmZansTd5RHyRxi6IYQClg+GbgS7f7zVyjalUqnT6SQSiUQiATi2nVpX7OrMJvbSQ8jfYeiGEApYPhu6OenYdmpdse/PSyOEPArLFBBCyEetPcBc3fMwFK+jKIqith/z9ngQQr4AQzeEEPJdiTu/YxhmMILD6A0hhKEbQgj5vsSdh/Y8DHChscnbI0EIeRuGbggh5JOa9i6/m2Zr+vKT7wEWzMGeyAgFPSxTQAgFLH8vUxjaUQ+rFRBCABi6IYQCmL+HbgghZAsnTBFCCCGE/AaGbgghhBBCfgNDN4QQQgghv4GhG0IIIYSQ38DQDSGEEELIb2DohhBCCCHkNzB0QwghhBDyGzxvDwChYFAL8C2ADkANcN7i/FyAyQAAsApglRfG5Utokx6Gdl+jTTqAkfux0Sb9iNeotXfGPjJ7eFwBBZQTlwmpob8h83hCZz4RIYSGgy15EXITFcBBgBMA3wKonPuUJwFWArwEMMWN43IDhjEbTQb2Q9rUb3WBbXRFG62vcYdoySxy0K6+7oHHjQ2fKwRqSHjXp+N++WPbi6umsWcsQ0AOh8fl4G/dCAUvDN0QcjkVwB6AvU5HbFZEAFkAOwFmuHZYo0UbdeTAzBhNZpocm8xGM2MavMAT4dd4+EXoZoU2Mnsr7zy9NGJuXMiIF7NhHwUUjysgJzkUj8vhk2NM8iEUeDB0Q8iFdAC/G0fQZiUL4H2ACFfc6i4zYzKZaLCYjmQYs3EwMjOa9Axjdu0TnWYbYYz3Xyc3hm4MA5QzIdGov6iybzsjw7lrlrj4504BxeOJyDGXw+NQXHLM54YAZvIQ8iv4dxUhV7kM8CzAZdfdsAjgc4CPAZ4c1acZTQaGMTNgNpr0YJEnc2eSzDJAGXO85Ve/RjoVt8Fov6iaejVtZOzFbeP9DjPAsH8A6CGvDPk1g6TxOBSH5O24HD6H4gFF8bnCMTwUIeQOGLoh5BK/A3gXQOfq294BWAvwFsA7AAMpEwYYo5HEZLSZMQKJ1YAxm+9Oa7oIGy4w7KOHmX3zq6jLVzW16s5c6ct5Ltbei6P4DlMAjM3PzkmOaz64HB6HwwcS4QFQwOFxhUCKNijsV4CQh2DohtD4vQxw0J33/53RdKZP/5HRBC6azSTv6yO+qVtfgKum3KdDQ5cf79yxfgqfN95vMsP+x9FF7ITvKII8k9loMhthmAwumZMdXHVH8bkiiuKwK/AQQq6CoRtC4/SGm+M2AAAetyZEsLVHWwTgzLwVNfCfgZWsjM2KK0yS+RbayJR+1Z6+OlISxvXQI6kRIraBPzQUdTflOhJ2TtZg1A59FIfHFZJJWJKo43C4bCEFQmi0MHRDaDz+F8AezzxJwDsRHpql0X4MAEOCMzvr5a3SLpgq83WlVe2r7g9PjBV5eyB3DfyhGa6OzTpp5yiwYxiz3Swdh+JyuQIOxeVyeBTF5XEEWC2BkDPwLwlCY/YVwLuefJ6AdyJEWNSvzxoSnDm7Xh75qKo6lSSMmzw33NsDGY2RknbkIscXmBmT2V5IR1bU8Tj8wXQdD2ddEbKEoRtCY6MDyPb8U0OFew300yZznOcfjdyh4Ya2obk/135pgr9jbD6kSAkFNWw2D2BwRR0NQ6I6LofP4fD4XOHglCvm51Dwwj/6CI2CyWw0m420qZ/PfZvPu+n5AVCgnyD6jVpb5vlHI5fr0NCfnOx6+5dTvT0Qz6BgMF1sGbcxDENR1OCiumEjOpOZNplpq4lXPlfI4fC5HB6PK+Ry+Lh+DgUJDN0QGhYbqJkZk8lsZN82uJxrocID3hoVn1cr4H1tMD7hrQEgl9Dqzfu/UL76dEyoMKjbalBk7tU2DTewnI5yEM/RJj0M7WaCwRwKBhi6IXQXbdQZzQazmTba/H5vScj/wpOjsiUS/HcAh26Xb3SqNLrrt9QAcPOWulc7pFndRLFo2uQJ7IcTQngz4ybyeZy5MyM9PdDxKa1qX/egJHYSruIaBmVnqRxDQr3h83N2gzkuV8CheHyuCLcFQ4EBQzcUvMiWULSp32Q2Gk16p/vZUiL+Z+4d2Uj4vBMcqt3MRHt3GK5C06a6n+6cvaRsuKpsaesZz60Wzo4BgLkzJvL43IWzfff7c+Rk56wpwiWzJ4x8KbJAwZD8HANADbQxGTY5R5v0ln2GuRwelyPgcQU8rpDLEeCCOeSP8E8tCiJmxmQ06Y0mvdFkMJr07DbqDlEUMGaLXQT4vNMcTotbxznymEAv4H+tM7zo3WGMX3XtzeofFA1Xla66IbkV+e+tJfGuuq1r1V3tbe027kjzszShD6LY/1h0n3MczJEaCLbzHGkjzOcKuRwSzOF7IvID+McUBTKGMRtNBtrUbzLTRpOeNIIfAUUxDGMxp8IwQ3cR4HHPu2Gko8bnntGBv4ZuNG36R93PZUcvKbv6bF+dOzMyZlLYtMkTyGSo7QWXb3bR9N2w+8LVDgC4fLPT8iQATBT7UJs0lqJdf/RH9VsvBElpgqfZBnMAwADDAY7dTSZIG2F2dQRFcfhcEcnJ8bhCDuWpDskIjQaGbijQ0Ca90agzmg3OzYFSQAGYzQxFsT1IHa+F4VK3XDLOceJ6O/M3ZtW1Nw9VXujWDNnvde7MyKX3T112/9S4ySO3N3MwE3qjRdXbT7e0abo1eh+cMNXqzSXH2nc+M3n8u10h51FAMUOCtmHTcgxjNhi1bE5usNxBwOeK+Dxf/E0ABScM3ZDfYxgzbdIZTXrapHdQWzCI9JRi7v5qzrBFbk7hcNrHPlbXoSifGMao9PXTH1ac+0edgj0zUSx6csWs9atnh4W4phJwZlwEOIztvIg2Mh9+2fZiSmSUGMsevcsqLTdsJDe4Z+tAbpjPFfJ4Ih5HyOeJMCGHvAhDN+SXTGaaNuqMZj1t1DlOrZEJFIYxswVrDHt6TDiUT6S7/C7rdvlG594//w9bhTBRLNrw+Jy1K+7h84PlLfDIqa6FCSFz40K8PRBkxTqSo4Bi7EVylhUPXA6PxxXxOAI+LwQ3e0AehqEb8hsmM20w9htNetrYP1KFAQWDsdrAAhfX7RZFUfqRL0JD1dbf+o+DZ9i1aI8uiX910yJXZdr8Qk29mjYya5ZEeHsgaETUkFVxDENRHMZ+Qq5XDwB6oCiOgBfC4wgxjEOegaEb8mlOhmsUAEMqQdlfoN22s6fZHM3lXHPTzZ3HMGJvD8FZZ3+6w8ZtfD73X3+5KDV5hrcH5VFNrb
ozV/pyAnO3q0BHWa6Tsz+1yjBmPd2nhz4M45BnYOiGfA7Zt4A26UbKrlGWHQEGKkE9MDwmzheSRf6yjenZn+4UlJwmcVvMpLB3/3WFM4UIgaRDQ5cf79yxfgqWJvi/u1OrDADFMEBxnAnjBPxQ3NcBuRCGbshXGIxa2qQz0FqHa9cGfgMmW+d4aGRDmc3TvPJcK2bGD0I3q7jt/V2PxkwK8/agPIo2MqVftaevjpSEBcuSviBBAdnvgf3tkaFg2DCuT9/F5fD4vBA+VyTghVJUUG99hsYPQzfkTSYzbaC1JMFmu5pk0JBwzeuJC6NpnreHAABgMs3y9hBGcPlG54hxm0bTo9H0AIBQKIyODsAWtaVV7avuD0+Mxb4SAY6CEcI4k9loMvTooAcAeFyBgBfK54ZgwxE0Nhi6IS+gjTqDSWug+4bvketb4Zol2riSASEFXi5W0NHPe3cAjnVrdO8NH7dpND3V33z72ZHKqmPVlp/1wb73Vqc+GhcXIO1qq+pUkjBu8tzgmiBGVmEch+IwzJAwzmgyGE0GABVFcQS8UAEvBFNxaFQoqz9SCLnJYK/LfoNRyzBm+xcNKTXwXRNCfiPif+rFARhN96n6vvDiAByjadNv9/3j8o1OAJgQwt/79mOWcdvFhkurVjzFfvjrN18DgA9+/yf2zJ//WrLmyVSXjCRaMpCbbFdfd8kNnddwQ1t1Tp2LpQlo0LDbrAIAgIAXyueFCPlh2DEOjQhDN+ReZLtA2tjPNii3RlFgNruvINQd+LxaSehmLw6gTyfrN7zixQE49mHFuWOnBopw39u50rJBLhu3/frN117Y9Ow9iTMtXzr40V8OflQGrovevBW6dWjovZ+3vf3LqaFCTKUgWwxQHBjmzZfHFQh4YUJ+GFY2oOFg6IbcwmQ2Gug+vbHXaDLYvcDxL6C+TxK6mc+r9cqjzUx0d88/GBB65ekjqjzeWPLZP8lxxvP3p62ew76k0fTcMz0JAH795mtvy35t99PfL/iAZODOXTw1/plTr4RuWr1592et256Mjp2EvSHQCBz8S8jl8IX8MAEvDJuMICv4GyFyJZOZ1upVqr5b3b0/9+m7bOI2iv1F06/jNgDo1eV7K3jq7ffao0d0+UbnwcqL5PjRJfGWcRsAVH/zLTn41x0Zw91hy0ubyMGPP9S5ZYjuV1rVvu5BCcZtyBkWLeMYqw35RvrnFAUvDN2QC5jMRq2+u7u3pbu3RavvHj5ic2OnXA8zme/p12d5/rkG4xMG4xOef64zaNr0+48HWu/OiovY9aslVhd8dqQSAIo/2isWD7tyPy5uKln9Ri72O0dOds6aIlwye4K3B4L8DDU4CcYwDAVWMZyxX6/GGA6xMHRDY8cw5n6DRt13u7v3Z61eZdOPjZ0KCJyIzVK/Pstous+TT2QYcW9/viefOCr/+ddzyq4+AODzuW++/JDV5qQtLbdJPemcOYmO77PikWQAqDpW3d7e6bbBukXd1d7WbuPTS3G3KzR2lMX+DcPHcC39evVI+wGigIWhGxoLPd3X09/e1aPo03Wy+zEPsojYfL5WdDwYEKq1Ze6O3o7++PSJhlUAwDBitbbMzESP9BneUV17s7r2Jjn+118ust0y4U5rGzmQTJQ4vtXUaQNVmXq9P20Xq2jXH/1Rnb0uxtsDQYFj+BiO7tN3dfUoNNo2Pd07fFNMFJiwrxsaBZOZ1tE9ekOv7W97FEUxA309Ajxis8Qw4p7+InHoZi6nxR337+6dVPJVtsEoKPnq1RULxKsWzozyyZ1LW9p6/vOv58hxavIMu1uU9mv7ycHFC5duXLvp4G4dHQPJtjutbX7R4+3UBc2Jiz0KpSEuir/7v1sBgMej7pkiAoDEWCGfT8VHC7HUFI2HZQxnGagZjFqDUUvpOkX8CQL+BD7XR1fBItfC0A05RU/36uk++w0+GAYoskojWCI2SyZznEZbFh6yk8c95/Kbh4l6//XpvUdOvdjSMVVeC/JaRfLc8BX3TVg4M9TlzxqPvX/+H7b7bsbzSY4v/tUvh61RsMJGez6uTWVsbNEBQNPtu2nChhvWf1nmTQ+JjxFOmciLjxHi/gpobJi7cxp3N2wga1f6DRouhx8ikAgFE6ig/Nc4eGDohhwxMyadoUdn0NhJs7G//AXiOrZRMZnjVH2fhQn/I0RY5No7C3iGFQvEixKTrt4WHP9nzw+Xe2ov99Re7okS81cniVOSxL6QyznyzRXSfRcA3nx5aVjICM2o/vzXktCQEAcXdHR0Zr6yEyxmTn3cRYVTIealn/sv/Xz3ysRYUXyMcHo0f2qkYG6co28IQjYG1qVY/cZsMtO9uo4+fZeIP0EkEGNnuECFoRuyjzbpdQaNnu61eYUCMMPQpD0CgD79b/TGx8NDdrpq8pRhxH06mY7eAACJsZAYK3pxVWRNveZ4vaZDQx852fn56a6H5oav9uwWmWd/unP5ZtfaFbMmikUA0NLWU370J/JS+rr75s4ceR/S+QvmOZ4GvdZ0gxwIRX4w+1NTr46W8F54dMhXRNNMU6seAK7d0RmNTLva2KGxKuKBplZdU6uO/TA+WjgvPmTF/Anx0X7wVSMfQeI2BoCiLFovDSbh+FyhkB8uEuBWbIEGQzdkTU/36gwam+IDtvogSCdGnWE0Leru/YeI/2mI8ACXc23M9zEz0f367Tp6A8MMWdoWKuQ8vTTi6aURDTe0p37qrb3cc+qC5tQFTewkwdMPSh6aG87nufdH09dP//7jH3r76QtXO97b+ShYTJXOiovY+Pi9DjaqcZAAACAASURBVD6XTaGpu9WOQ7fbt1rJge8vdGtq1Z250pfzXKztd952UvtyS39zm17RQSva9Ip2679fina9ol1fVaeKEvOXzA7DGA45jwIgcZvVv860SU+b9Fp9d4hAIhKE4zapAQNDNzSA/KKmp3tstoRn02zIKTp6g47eIOB9LRKUC3gnrF596p1vAODLdx+3+7lG0yKdIV1PSx033V04M3ThzNCNj0w8Xq85ebGntctQUtX+l287k+eGP7FY7L5msNdbVL39NAD09RsAoPJ4I5kq5fO5u371C6tuIFaiY6LIQWNj0/yF8xxceepkLQC89MrgVmPf/Fv0hnI71730UfveVaP+GlynQ0OXH+/csX6KkxHz3LgQy4nR4SK5Dg1dVaciW9cvSZzw0NwwnE5FThr8gzhkJZyZMfXpu7QGVYhALBKIcY/UAIChGwKT2agzqHWGHus50IEMPKbZxoLtncvn1Qq4/+Bxz5Njq8uMpkUMIzSa76ONS2lTslWazbEoMX/jI5EbH4msvdzzzVlNU6uupl5dU69OjBU9vlicPNf1syQ3WrrJwb0zIpVdfezGCenr7psZN0IzM7E4/NdvvvbB7//02ZHKZ5+XDndZS8ttshFW2jPrBk49/u/t6n+3vOb6hxseevvsg/fOtP10j6GNTOlX7emrIyVhY3wjtIrkGm5oz17T1jX1qvsG1pWq+0zkB4oxHBqlgTkSy122GMas1au0epVIEB4iiOBy8N3fj+EepkHNzJi0+m6docf6BSYwm+j6gq27rwHAoZx7XH7n1i7D12c1tZd7tHozAEjCuI/MD1+dJI4Su2yp8u8/PvOPOgUAvLpp0cm6Ww1XlQAwKy5i71uPOfPp7N7zH+x7b8vWTXavIXuYrlmb+ueKkmFu822O5JX/Wpr3wzcvzxrpie7bw3T/l8pF94S4Iz6+3NL/w+U+yxiOFR8tfGqpxB0PRQGMAevtGQAAAzi/hqFbkDIzpn6DWqfXWGfaMGhzM/eFbgRtZE5e1Hx9VtPaNbBbzpLZE1YvDHdJP5Hs/KqWth4AWLvinmOnrgEAn8/dnbN6xJQb68M/lr4jew8A3i34bdqz6yxXs11suHTwo78c/KgMAGrrqu9JtJ9Uq945a9NB+JdPr++2P+c8hJtCt6o6VWePafOqkWsyxkPRrj91sbfuap9ViUOUmP/ssggPLG1EgWXILCqBAZyfwtAt6DCMud+g7terhwZtAwvavDasoOHu0I3V1Koj/URoIwMApJ/II/PDxzzBR9Om5974m9XJjY/f+y/rF47qPmz0R
rxb8Fu1WkMmSQFgzdrUt//t18Muhrv+8bpF+T86vcrNHaFbww1t1Tl17nOea1xCYriTFzQkn0pIwrhPLI7wkQYxyJ9Y1KISXg/gGhsb77132CKnrKys/fv3e3I8vg9Dt+DSr1drDSqGMY98KXIPj4VuhLrPdPJiD+knAgB8HjXmfiKXb3S+ufu45ZmYSWFFeU84rk6w62LDpcr/c4wN14g1a1Of35iW+vgqB5vTV++cteng4oJzn24fca4UANwQunVo6L2ft739y6meD5i0evPJC5ovf1RZTqSGCjkpSeInFkeMOSJHwYmirN/9Q4SSUEGEV6pQa2pqUlNTh3tVKpVWVla69ekAkJKS4r5HuByGbsHCYNT26bqsdohnGIbC6VHP8nDoxmq4oT3e0FN3daBRX+wkwROLxY/MFzs/6VZ5vLHks39annlv58qFs6MBQNnVV1t/q/afd/h8zr9uWhQzKczJe7a03CYHQqEwOnqk+cdv/i16Q/mD7x8/+mqCk/d3beim1Zt3f9a67clo99Xwjog2Mj9c7vnbaZXlLCqfR6UkSZ5ZNhEzcGhUrAI4iuKECieGCDy93R4bujU3N9u+KpFIJJIRNj4eM4VCkZCQQB4dHx/vpqe4HM5wBz6T2ajVd+npPsuTJGjDuC14kH4iHZpJbD+RQ3/vOHKyy/l+ItdvaSw/XLvinsmRoZXHG6t/UFxvUbHn/17bvHndfU6OajSd25oP/Ec5QHqO03Gby5VWta97UOLFuA0A+DxqxQLxigXiUxc0VWc1pKsIbWSq6lS1l3teWDFpxQKf3OYW+aTBuG1wbwbG3Kfr1NM9YaIor2yH6kfBk3dh6BbIGGD69SrrZW0MAxi0Bavx9BO5YRGfAUDDVSWpVLA0IYS/eN5k148b4PqHObIz8OD7mcNOq7jZkZOds6YIl8ye4KXnWyMBXN3V3m/Oacj+Wuo+U0lV+6mferc+FuXd+BL5myFNoIwmg7rvtkgQHiac5ONdfBUKBTgR8CmVSp1OF0hxoU//VNB40Ca9qveWVq9i47aB+nAM2hBA8tzwvM3Tfvfy9JQkSaiQ09Sq2/+lckfRzSMnO223bAIAmjZdHxq6kVJT1rKkabt+9YvS/73Omb2wRu/b/3z7LCzN+5OXUm51V3tbu41PL3W2kNZjlsye8NYLU3Ofi2XXul36uT/vcMsXZ1SOPxGhoSgYWn2qM/R097YYjFpvDciBmpqa7OxsiqISEhISEhIoisrOzm5sbLS9Ui6Xp6WlTZ482fay2tpacgfyIbmAoqi0tDTPfSVjhWvdAhBJtmn1lv92U4BbjvoGb611c0CrN9de7nHcT8S2RoFYljTtoYWxyUnTRtx1fjxG1RDEkkvWuina9Ye+6Xjrham+3IyDNjKfn+6yjNgWzgzNWBOD5Qto9Ia8X3gg/caudXMmIMnOzi4qKgKArKysxMREAMjNzSUvXblyZc6cOeyVu3fvJi8VFhYCgEqlKigoKCwszMnJAYDa2tply5bZfYTvx0XuCt3IfFxlZaVUaqdtulqtjoiIAIDi4uKMjAy7dygpKcnMzGS/y8hJtEnf298+pBwBW7X5Eh8M3Vikn8ipCwNr2iz7iVjVKHgmYhswyoYglsYfumn15vf/envnM5Nd2Nl4YEjtnXq93rX7tLZ2GUq/amd3tQ8Vcl59arJLWvqhIDNkEx0OxZ0QEiXguesP0qhCN4qiysvLU1JSYmJiyBm1Wv3WW28VFRVZthFhG460tbWxVyoUiqamJstiUj8tU3BXHJ2VlQUAZ86csfvqzZs3yYFcLh/uDpmZmQBgGUG7XE1NDakKDhhavUrdd9sibsMZUjQKibGijDXR+7JmbHwkMkrM79DQR0525pQ2l1S111/thsFZ0Yrfr/9txrLU5BmeiNsArn/15Y8AcPCVaMmsof/7t2o3P5o2Mh9+2fZiSqRr47aTJ77/1aaM+xIfXDR/RbRk1od/LNVoegCgvb2TfGnkwzGInSR464Wp7MSuVm8u/O9WnDxFozfkvcPMmDTatj59l3ULd5c/1cbu3butrmEYZtOmTWw0BgASiYSEHCQbR7S0tABAYWGh5ZXx8fH+1QRkOO4K3datWwcABQUFarXa9tVLly6RA7lcTpYZWmFPJiUluWmECoUiNTU1NTXV7gD8DsOYe/qVWn235SmcJEVjIAnjPr00YndGfO5zsQtnhtJG5tQFzSWlaN4Dv0heet+yRfGeidhYs179tF193d7//t3dJQtHTnUtTAhx7c6h7xd88Fzar6qOVa9Zm/puwW8B4B3Ze69m/hoA9PqBTegddLYbEZ9HbXwk0nL125GTnYf+3jHugaPgM/RNpF+v1vS1msxGTw6hqanJmcsmTpxIDtg39NDQUADIzc1VKpVuGpsXuavClO2MfPPmTdvw68SJE+xxU1OTbZayvr4eAKRSqR8lML3IaDL09CvZZNvgzsOYbEPjMthPhD5erzl5kdumNo22n4hfq6lX00ZmzRJXliacPPE9aUR87O+f/eLBRQDw6uvbDh+q+PWO3/7tM3lUVCQA/PrN18b/oIUzQ/O3TN/3f+6QydOaenW7ht6ZNsWXl+shn0QN/pcBUv3Wd2uCKErId7Z346jY9nUbc0e3efPmSaVSuVy+bdu2Xbt2BUayjeWurBs70ckm2FhqtZpkNcnKwePH7ax9JjOtdtfJkTsoFApnQmmlUhkYSTUH9HSv5SQpBRSm2pALkX4i+7JmZKyJTowVafXmmnr1Wx//nF92i10VF3iaWnVnrvS5fJfSA/s/BoAP9r1H4jZiy9ZNa9amfnak8uZNBQDMnTfbJc+ShHHfemEq2/Cl4YZ2b+UdsisaQqPEsFOlZIanT9/ljsfE27AbuikUioqKirS0NHZelS0UZUkkktLSUhK9paamUhRVUVFhdxrQH7mxZqS4uBgAysrKrM6ThW4ymWz58uUAUFBQYPu55OTSpUutztfX12dnZ0dERCQkJEyePJn8MOw+PVBLgq1o9aqe/naL9QeMu9cioKC1YoHYqp9ISVW7g34i/qtDQ5cf78x+arJrc1QtLberjlUDwMPLH7J6actLm6qOVf/z/AUAmDMn0VVP5POo7Kdi2KVvDTe0+48G4OQR8gCrpEC/Xt2r884sfEVFRUJCQnp6OgAUDpLJZLZXxsTEVFZWVldXk5Vw6enpERERZE7P37mxJe/ChQsBQC6XK5VKy3WCJA+3dOnSefMGdphubGy0LEdgY6wZM2ZY3rCiooL8tGQyGSlQzc3NTU9PLysrO3z4sGVsbrckODEx0XGxqoOaCd+k1XcP7QACuH88crfYSYKtj0VtfGQS20/kizOqL86oFs4MXbNIEgDFjLSRKf2qPX11pMvbaty4dpMc3JM40+qlxMRZAHDwozIAmJ4Q59rnbnwkksflfH66CwDqrvaWVHEy1kS79hEoGFB3/58BAJ2hx2w2hYfGUB5831EoFCQMKC8v37Rpk+V5u2kgAEhJSUlJSXnjjTf+8Ic/FBUVPfDAA5Y1p37KjaEbG5ldv37d8ttEFrrde++9pCqkqKjo7NmzlqHblStXAEAmk1lGYzU1NeQHdv78eXbx3JYtW7Zt2yaXy0tLS9mwrLGxkcRtlj+ejIwMdrVjcnIywzB+WhLM6tV16AxsGRq2bUMeFSrkpCRJUpIkTa26b85qai/3NNzQNtzQWvYT8fYYx6i0qn3V/eGJsSJPPlQoGth06KVXNo+nRmE4zz480Wgyk1LTUxc08VF8167hQ8GEAYoChgEAg1Gr0baJQ2I8tukCuw7eMm5zxpw5c/bv308Wa124cMHfl7658dvN1us2NDSwJ9mFbiRWI4WollULAHD06FGwmS3ds2cPAFRXV1sWPcTExOTn5wNAbm4uO4cd2CXBhGXcRlEYtyGvSYwVZT8VY9VPZEfRzZKqu93F/EhVnUoSxnW8IZhbPbZmtZvuvPGRSHZ70yOnuvzxp4N8BcOweTba2K/Wtnpsoc6dO3fAXuuJ27dvj/POOp0//Y1wb6RMIjPSoY0gs6XstDQpRC0qKmJrDtjYzvJn09jYSGYzlyxZYvUI9jK2V1xglwQDgFbf
fTffRuF+GMj72H4iO9IGGsCeuqDJL7v11sc/19SrtXqztwfolIYb2obmfpeXJjhD3T3wm+f8BfPc95SXUqNINpE2Mvu/UPrLzwX5IMuuIUaTobe/3TPPJQuxCgoKLAsQ7e6LUFFRYdW3lV3lFhd3d00CO+fGppD8opTBvaEb2yKE/S6TDBybUWPnSa9fH+h1TiIwq7YgJJEGg7WlVshLnZ2d5ICUBAPAtm3bAqzjLgDo6d6769sGs9YI+YglsyfkPhe7OyP+6aURkjBua5fh0N87ckqaD/29g91lyzd1aOhPTna9+tRk9z1i0qSB1lPt7Z1WLzU2OtW8apz4PCr76RhSe9Ghof/yrfUwEHIeBcAm2/R0n0tqTm1b8hLsBcnJyeT9PSEhYffu3XK5PC8vb9myZbZlCrdu3UpNTU1LS9u9e/fu3bvz8vIeeOABACgsLLRq9U9KKjMzM7Ozs/Py8shKeh/nxrVuYBGZsc3bSPKMDekAoLCwMDc3t6GhITk5GQbTclZtQdj8mW0BsC1SEkzWwJHHlZeXr127dsztYXwHbdT19rNFPRi3IR9F+ok8s2zSD5d7jv+zp6lVV1OvrqlXJ8aKVt8fzk7b+Q6t3rz/C+WrT8eECt342yxbf3C27vyaJ4e0E/7sSCU5UHerXbs1lpUoMf+l1KiSqnYAOHVBs2axOD5a6L7HocBGwd31Ov16NQe4IcKxvM9apsFsWcUDpaWlhw8fzs3NZbcuJVtu1tfXy+VykWhgleqWLVsAIDc3ly1AlEqlGRkZtk3HMjIyNBpNbm4umfEbriuZT3H7dBvZilQmk+Xn5yuVysmTJ8PQfcrYVCc5SXaWtaxFYG8CANXVw+58Y7uUraam5siRI+zOGFb39LsyBTNjUvXeMjMmbw8EjYsv72HqJq1dhq/Pak5e1JCmYpIw7iPzw1cniV2+K6gtJ/cw3VfZtnxe2JLZE9w9ng//WPqO7D0AOHfxFBuivV/wwQe//9NLr2w++FHZB/ve27J1dOuvx2BfZVvd1V4AWDJ7wo40NyYaUTCgLNbtSMKm8rme+GVArVaTmc0R377ZqTnHV5IbikQivyg+dXvoZhmZkWOrHeXZeK65uVkkEtnGdjDKvWmtNDY2kpJgsNmG1r9CN422zWDUensUaLyCMHQjtHoz20+EnPFAPxFnQrcjJztDhFy2+Zlbtbd3vvH6W6S726/ffE0iEX//3Q9Vx6rfLfjtylXLV614ipzv6uz65ebnf/HgIoDmA4+vlg3sBZ1e4aK9vxTt+rz/GliFkv8vcZh4Q+PE5t64HF7EhDhPtgsJTm4v6LVs3kYWui1atMjygpiYGJKfrK+vJyvebCetSeUBjGn9ICkJJscXLlwY7af7iH6Dho3bcJYU+SPST+R3L0/P2zyNlHA23NAW/ndrToniizMqdZ930sl1V3tbu42eidsAIDo68g9//B3ZuvSD3//pHdl7Vceq//zXkldf3zZ/4bzij/aS8wc/KgsRiQCgeudqGeT9QPZsPZe4+/GPHSUPnRYfLWRTjMfrx7jVPUIs9l3JZDZqdW7ZaAFZcnvoxrYIOXv2LJlyTky07hW+efNmADhz5gyJ7Vavti6PZ+O/uro6l4/Q90uCrf4y4K8zyK8N109k/5dKD3esULTrj/6ozl7n0fmR6OjIV1/fdu3n+nMXT527eKpdfZ1d9/bs89Kfmn48d/HUtZ/r5y+cB/Dt0YMA980cyBzOevnoNy/PctEw1j04sCaprqnXRbdEQW4gfrNMNCA38UQbPdIipKysjIRutrOTJDIrKCggFyxYsMDqAolEUl5eDgCpqam1tbVWrzY2NlpuhxV4JcFafRcp5MGgDQUMtp9I9lMxZM609nKPJ/uJaPXmkmPtbMWlh4nF4XFxU20rEqKjI+Pipg525V217iWAg69E7/zW5QNIjBWRtYbqPtPlln6X3x8FH1K0AACg1Xd7dygBz70VpgSpJyVhGdmZygpbPSCXy6VSqd1Fgps2bbp48WJBQcGyZcukUunKlSvJeVJjYtlb+datW+np6ew1ZBcsGKYkODMzMzMz8+zZs1FRUQUFBT7YI81oMujpPnLsc4NDaNyS54Ynzw3v0NDH6zU19RrST6Ts285H5oufWCyOnSRwx0NpI/Phl20vpkR6oFRiPFL3Xv/h3g0Pvf1K9EFw4Vo3InluGNlfoeGGdm5ciOtujILV4BsoedsS8sO8O5wA5qGGrmlpaSR0O336NGkCYoXddZRU+Q53H7lcXlJSYrXZaHFx8cqVK9mwTKlUksph9oLhSoItn0suq6ysHPXX5mbsxgkUdt8NCEFbpuAM2siw/UTIGdJP5KG54WPLjQ1XplD2bWdkONefNoO6/vG6Rfk/ujR6q7vau6+yDQCS54ZnP+UHVXXIDzAMyb3xuIKIsGneHk3A8stoQKlUkgVqjut4A6Ak2MyYunt+9tgeI8gDMHRzRmuX4Ysf1T9c7hlnPxG7oVtNvfrndnrrY1EuHLAHXP9ww0NvQ8G5T7e7aL0bW2eaGCvK24zvssjFwkNiMPHmJp6YMHU5J2MsJ1t+SCQSn+3WqzNoBla5YcoNBZPYSYKMNdEvropk+4l8cUb1xRnVwpmhqxeGj6cBW1Or7syVvpznYl04Wne5/vG6RU05A2m25m/+dhYgPdFVdQoAktCBf/9VXirvRYFpcJsfPd2LoZub+GXoFjzYOh2z2Wy5GQhCwYD0E0lJklxu6T9e31N7uafhhrbhhjZK3LU6SfzI/HBJGHdUN+zQ0OXHO3esn+KV0oRRm/Xyn97f8JCEDdZcvNYNIbcYzDLQxn4GGOzx5g4YuvkuM2Mymgaal2LchoLZ3LiQuXEhm1dFnrzYc7xeQ/qJHDnZmTw3/PHFYrKl+ohoI1P6VXv66sjRBnxeNOvVT9tfddfNm24PFJZG+M83BPkRBhja2C/gubHndtDC0M13GWi2NQ6F1aUIkX4iTy+NqL3cc+qn3oYb2trLPbWXe2InCZ5YLE6eG+54B9LSqvZV94c7GecFg2/OacjB3On4PUGuRAFFlvrQJh2Gbu6AoZvvMjPGgSPGDJh1Q2gQ20/k67Oakxec6idSVaeShHHJLg4IAGov91z6eSDrtmI+fluQK5mZgRU+JhPt7bEEJgzdfNfdneYxbEPIRpSYv3lV5MYVk9h+IjX16pp6td1+Ig3N/bl+UZrgESTYJccpSRI3Nc9DQYtd4XM3AYFcCkM332Uys3/oMXZDyD4+j1qxQLxigVjRrq86q/nhck9Tq66pVffJqa5H5oeHREzuV7UBwKtPTfb2SH1Fa5eh8LM7ZL+K2EmCzasivT0iFLDMZixedgtPbISFxsZsxt9XEHJWfLQwY030vqwZWx+Lip0kUPeZvjijmvnQc+RVx8vggkfDDe3/LrvVoaEBIFTIyX3eT4ptkX+6O3eEXAq7hfmu7t6f/71Cwa5HQQg5qaX+a5XiYuSsRa0Xv138wv/y9nAQClJv/3Lqivvu8/YonOJkD3+CdPJ3fHFFRcWJEydu3bollUrXr1/v2p7/GLr5LrX2Dm3sBwAGZ0wDCO6m4D56nf7gR3/Zt+eAsq2dPXnoL0Xrnn7Ci6PyBYp2/aFvOtjtxSRh3B3rp2CxLXI3iuJEhid4exSOqNXqY8eOpaens2cc7JwJALW1tYcOHSoqKmLPFBYWbtmyxSoyy8vLKygoKC4uDg8PJ5uqHz582IXN/3Gtm+/iUIPNlgZ3hUMI2aVsaz/4UVnx/o/Vag17UhQ64fUd/8/yR+xsmhw8tHrz56e7q+pU7JmFM0Mz1sT4UXM75L84lE8vVFAqldu2bSO7ostksoiIiBMnTsjlcrlcLpPJ8vPzra6vqKhgg7zCwkJycW5u7okTJ0pLS9norb6+vqCgoLy8fNOmTQAwY8aMZcuW1dXVpaSkuGrkGLr5Lg5n8N9WDNsQGsbPipYD+w8e/Ogvep2ePTk1blrisucO7c2eEBa8iSWt3lxVp/76rIpUJABAqJDz4qrIFQvE3h0YCh4czug2Hfawd955Ry6XS6XSP/3pT2TqMycnRy6Xp6WlFRQUrF692jLYqqmpIXFbdXU1OZ+
Tk6NQKF577TW5XP7OO+/s37+fXNnZ2QkADz/8MPlw6tSpAHDu3DkXhm4+HREHOWrwp+Pjv7gg5BVXG6+9/upvFi989MCHH7Fx2+w59xTu+//T/t/yAx+8FrRxm1Zv/tv33TklzZ+f7mLjtiWzJ/zu5XiM25AncTm+mx6qr68n855s3EZIpdLCwkIA2LNnj+X15MPi4mLLCCw+Pp4k54qKiurr68nJyMhIAKiqqiIfNjU1AcDy5ctdOHjf/bYiAS9Uq+8GAFyPiJCl707VFu8/ePSLry1PLl+RnJn90uNPPr638s6LSyOixD79676btHYZvj6rOXlRQxvv/qMRO0nw4qrIhTOxqT3yEHb/Hx5HONrPVSqVOp3OmVqBcbp06RIASKVS22dJpdLc3Fy5XK5QKMirjY2NZF51/fr1VhcnJSWRgzNnzpDjpKQkmUyWmZmpUCgiIiJyc3NlMllysitXbmDo5rt4XAGH4mJxNUKso198Xbz/4Henai1Prnv6iczsl5avSAaAsm87FyaEzI0L8dIAvab2cs83ZzVsIQIRJeY/uywCM23Iw9jfGwT8UfzCUFNTs2fPHhIhAUBhYeG2bdskEolSqZw8eTIAqFQqF670v3XrFgCsXLnS9qU5c+aQg6amJhK6XblyBQCkUqndQtHCwsLc3NyzZ8+yZ/Lz85cuXdrY2AgA7KI3Fwrw0I2U+0okEhf+vAnP1AYL+KE6Qw9YbAmHUHCq+Mun+/YcuNp4zfLkphc37Ni1ffacgXLdmno1bWTWLInwxgC9o0NDH6/XnLzYo+4b8jtefLRwzWIxBm3IGwaaIpDsg5OfQ0oyAUAqla5cuTI3N5cs/6+srNTpBiujR3ofZ9+XHRhbPo8EYXbjPBgM9YqKitjlbgAwXI2qSwTmIqr6+vq8vDyKohISEhISEiIiIrKzs9l5aFZtbS01kpKSEtvPys7OjoiIIDenKGr37t1KpdLqsry8vPT09MWLF2/evDkzM3Pbtm0j/pGyxW7ci3EbCk56nf7Ahx8tXvjo66/+ho3bhCLh9ldfOdvwjz9++B9s3NbUqjtzpS9I9gZQ95lq6tV5/9WSU6L44ozKMm5Lnhuet3la/r/EYdyGvGSgsE7Im+DkJ9TU1JC47fTp05WVlTk5OQzDFBcXy+XyiooKslZMJpONeJ+6urqEkdTU1JCLxWIxAJw4ccLBDc+dO0cOVCqVg8vCwsKc+0JdJgCzbiUlJZmZmVYni4qKioqKKisrLQNhrVY74t00Go3lhx6uDRbwQnlcgdFkAACgKMBFbyhoqNWaAx9+bNXvQyIRZ2a//NIrm2MmR1te3KGhy4937lgf4HsDqPtMdU29Z6702XbqloRxH5kf/sTiCOz6gbyIYRiygSmH4oqEzv7ywFYAWC4Iy8jIkMvlZWVl5F17/vz5I94nLi6OVBg4voYcGGALZwAAIABJREFULFy4EADkcnltba3VQjTbPAuJLEfEro1ztwAM3TIzM6VS6a5du5YsWULyqzU1NampqQCQlpbW1tbGBlgpKSnNzc12b6LT6e69914AmDZtGnvSK7XBIYKInn4lAGDchoKE3X4fMZOjd+za/tIrLwpF1gufaSNT+lV7+urIQI1a1H2mhht9Z6/3113ttX113vSQFfdNeGhueGCHrcgvsBvPiwThlHN9rRQKBVnfZjsdmZGRkZaWRt6F582bN+Kt5syZk5OT4+RQk5OTpVKpXC5ftmxZeXn54sWLRSLR7du3GxoabLM/viYAQ7fTp09bRdApKSnV1dUkertw4YJVZa/dm9TWDqyDXrt2LXvSQW2wXC4vKirKysoiBSZsbXBGRgaMrzZYyA/rNwwk3nDFGwpsVxuv7dtzoOIvn1qenD3nnh27tm96ccNwn1Va1b7q/vDA2xugqVV3rqnvnzf6Fe1621fnTQ9Zem/YksQJgRqwIr/DAEPCNQookcDZlBt5fwSL4gAWSaCQFh4zZsxw1ThZpaWl77zzTlFRkeVuCgCQlZVFnmuZu/EpHgrdPFbuCwB2S3ATExPJgZOpr0OHDgFAYWEhuy7Si7XBocKJGm0b4Io3FLjs9vv4xYOLXt+13fE2VlV1KkkYN3luuJsH6CEkwdbQrPvnjT62JZsljNiQz2LTbCFCifMFCg6IRAO/j2VlZbm81hAAYmJi9u/fn5WVpVAoGhsbxWLxPffck5iYGB8fn5aWRi5gB2C5+dVwPBPkgAdCNw+X+46IrEx0jO3UZ7kwzou1wQJeqEgQTkpNEQowI/b7cKDhhrahuT/3uVh3DtDttHrztdu6yy39wyXYAGDJ7AkLEkQYsSGfxfZy43OFIUIXV3mvW7fOmcvYDIsDGzdutAqwkpKS2PwLi9yHXRjHZn98hHtDN98p92Ufd889I2/7/fe//x0AsrKyLPO33q0NDhNOMtDawR5vFGD6Dfm/ir98emD/wQsNP1me3PTihu3ZLy1YeN+In96hoT852fX2L6e6bYBupO4zXfpZe/2O4ZJi2HAtSsy/f2boggTR/TPDcB0b8mXsGxIF1ISQaCdXuY2ou7ubHNiGVna1tLTk5uY6vmbRokUjBgzk7R4ASIIJBpM+ubm5dtfSkUJUZ2pgXcWNoZtluS+ZK8zJySHlnxUVFSR35WS5L1mm5gBbNzAcNh+2YMECx7dSKBTkZ79161bL896tDaYozoSQKDJtinEb8mt6nf7gR385sP/gz4oW9qRQJEzfvGHHG9unx8c5cxOt3rz/C+WrT8eECv2mw1GHhr6s6L90S3/ttq61yzDcZYmxokWJYffPDImPHnUneoS8gbGYKo3gjnLfUrI0HACUSqXVpBbZ8MB5CxYsqK6uHvGaEe9DUm6WK6ZIOardQcJgh5GlS5eOarTj4cbQzSvlvnap1WqyCLGwsHDEvrhk3zGpVGq1NM3rtcECXmiIQNxvGGiUwFj8bUHIL5B+H4c+LlO2tbMnh+v34VhpVfu6ByWxkwRuGKbLqPtMCqW+qVV/paW/Wam3u3aNiJ0kmDc95N444bzpoTglivwKwzZyE/BCQ0c/VcrWH/zwww9WU1VlZWXkoLu725k31piYmPFv8V5fX0/SNxs3bmRPsvWtNTU1VsufamtrSaj30EMPjfPRznNX6Oatcl+7PvnkE3Jg+ZOwS61Wk6pgUhnqa8JEkWbGpKf7YGBB6N2/Mwj5MmVb+749B+z2+9j04gaJZHTNY4+c7Jw1RbhktrMNPz2GrFpratVfu6NTKPVWOxxYSYwVzZ0umjVFOC8+1I9yhwgNNfAexOMKwkNHvWMQAEgkErJYPC0trbm5mQ3R8vLy5HI5qQ9gSwBdq76+3uq2bCux4uJiy2CRHWR6evqMGTPYzE5jY+N7770HziWGXMhdoZsXy32t1NfXk2issrJyxLD92LFj5ODRRx9198DGZkJItNlspE3k/Y9i+x8i5Jvs9vuYHh/3m7d2OOj34UDd1d7WbuOONJ/YNUHRrm9X0Yp2+ucOQ3ObvkNDO7iYz6PmTg+5Z4poXrwoCHdZRQGMy+FJQmPHPBG0ZcsW0t8+ISFBJpNFRESQDwsLCx977LGioiLSrqGjo2Pr1q3JyckATXuXz971PfnszKPMgbWOH2CPQqF44IEHACArK4tUIbDr5AoLC23TN9u2bSOjWrZsGVm+39TURCIZmUw2zgTTaHmhr5u7y30tsT+bwsJCZ4oGSHq2uLjYdmA+UhtMASUOnaLqu2UyG4H0P2QYwOgN+Z7/+fHcH/ccsOr3sWDhfW++tcNxvw8HFO36oz+q33rBO6UJrV2G250GEqi1q+jhygtYfB6VGCu6J1Y4PVqQECP08eldhJxnOeNDURxxaCxFjT1zHBMTU1paevjw4dzcXHZtErv7UXl5eXp6OjlPOq4d2z57F+y5yuxMBICmvcuX75393c7RloDGx8eTRJrlO7tMJtuwYYPdDJ9EIjl8+PAnn3ySmZkpl8vZUlZ37C4/Im+25HVruS8AKJXK1157DZyOiGtqasiD1qxZY/uq79QGk78nGm0rid4AozfkY+z2+0h5bOWON7aP2O/DAa3eXHKsfeczkz1Tbnm5pb+v36Rop+900x1quq
lV58xnzZseEh8jjI/ix08WYp0BClB3V1pTFEcSGsvljDeWiImJycnJYTf7tnxD37RpU0pKik6nk0gkEokE4NjnxQCZcwbekhN3fvfdGB+ak5NDtkQiHw7ef1gSiSQjIyMjI4P9FI81crPihdDNM+W+SqVy27ZtcrlcJpPl5+c786AjR44AQFZWlt0fhk/VBnM5vIiwaWptK7u9KfYLQd7S2mUo/aq9qVX3zLJJ+hvVtv0+nn1euuON7c70+3CANjIfftn2YkpklHh09WvOULTrtXrzJYVOozW1dhludxkcL1NjxU4SREl490wRxU7iTY0UYKyGgsDdjBuXwxOHThltSakDwwVPQ5eRrX0mE4qL11EwtplSa2MIv7wVsbHcFbp5t9x3DHEb24aX5GNt+VptMPldh43eGADs94Y8TKs3f3mm+4szA31zjnxz9ZsPfsO+Otp+H44dOdW1MCFknEvEOjR0h8ao6jW2dhmNJvO1Vr2qz+SgT4eVKDE/WsK7Ny4kWsydEikIvK23EHKMAop9j+FyeGJX5NvGYO0B5up9y2fvWkcVw5jXuvk1d33TvVvuS+I2qVT6+uuvO/kpn376KQBIpdLhcoG+VhsMJHoLm9qjVRqMWgAgcRsWnSLPqL3cU/Ztp2V26kLVwJIRiUT80iubM7NfHlW/Dwdq6tW0kVmzxKm+A+o+U2u3gaaZplY9AFy7ozMaGce9OexKjBVJJvCmRwmixdyoCD4WFqAgR1lsxsjl8MWhU7wStxGJO79jdg6UK7gs/+Y/3PV992K5L3kEAPz2t7/V6XTsnDTLNl5UKBRkCeSuXbuGu62v1QYTFFDi0MlafbdWrxo8gy3fkHu1dhkO/b3j0s/97Jmu5voLX+7t7fh5zP0+HGhq1Z250pdjsdvV5ZZ+ACDJMwC4002r+4zkSto4lsRzfLQwTMS5Ny4kVEglTBbGThRgczWEWKSVAftXS8ifMEEUOZ66BJdJ3Hlozyezd11obIK1vrIc3RMohnHXFBs7awkAtuW+pPBTJpO5vNw3ISHB8TW2Wy+QPR5gpA1V1Wr1li1byFdkWxvs5Mysm+jpvl5dB8OweQWcPPVRW3dfA4BDOSNvyOaDrGZIAUDf2/VT1f7Wi9+Sfh/PPi8Visa43qupVWc0DfyhvaQYqAnQ9BtrL/VOixLwONQYMmdWYicJIsK4kRJ+VDiPRGmSUC4WfiLkCKmEGxQmnBQi9Ny243Y07V0++yfZQIRAwoagmzN1Y+gGAEqlkpT7smfYct+KigqywwEAnD9/Pikp6dh2at2FPVe/Gyz33QqHRl/uy+5q7wC7MReLtEZzpsRXrVaT2mDLk16pDbZlMht7+5WDLd8ABn9V8uKQkC3/Dd3qrvaWftWqNdz9VVvVcunadxXTp0U9tyHtkZXL2PO3Ow3qPusYq99gViitu2mMPxqzReIzSRhvykQ+n0clThUCwD1TRLgNKEKjYrX8hkNxw0Mn87ner8WxTPME51o394ZuBLt/vG0RqGW573ZqXbH//Ai8XhtsFwNMv17FTp4CWP/ChLzOH0O31i7DHz+7ekvjE6vyJWHcqZMEABAfIwwRcABgXrwIAEKFHCzwRMhVrCZuBLzQCSFRHAoXEvgET6wx9Eq5r7v5VMTGooAKFU7k80J7+9tNZhoABuM2rF5AY/Txf1VWN8dyQye59SkkVcYei0O511p1Wr35qQclYSFcAOBxKazoRMgjGLCoJKUoTphwkkgQ7s0RoaG82ZLXCpb7ugqfK4yYMG1o+g3jNjQ6ep2+vOzTfX848LOihS+aMGv5L2c+9ByHN2RZGFndb/u5bD6MxU5cWooS84Zr0tZwQ3vtju7/2zxtfF8EQmi0hrxZYLLNN/lQ6AZBX+7rQnbSb+Q85YkpcuTX1GrNgQ8/PvRxmbKtnZyhdb13/ufI40nhkgXPnLt594+TWmtcMT/CyZ4dzuvQ0J+c7Hr7l97Z7QqhIDWwuoYZ/AiTbb7Lt0K3AcFa7utyg+k3db9BTYpPSdyG1afILmVbO9kqXq3WsCdjJkdnZr/80iubSb+Pplbdpye7SGcQdZ+p7NvO4//seXFV5MKZoS4Zg1Zv3v+F8tWnY0KFPtB6AKFgQLZStPitPkQoCRVE+ET7D2SPz4RuQ8t9v/zke4stytDYUUCFCiNEgvB+varfMPB+jHEbsvKzouU/frfvb5/J9bq7RaDT4+N2vLE9ffMGy34fibGit16YWne1t+zbrg4NDQCtXYbC/26dNz1k62NR42+0UVrVvu5BCTbsQMgTSNBm0YhAwAsNE01y4d5WyB18aPoMy33djTbptbpOy+4hAIBb13uYr1WYXmj46fe/23f0i68tTy5YeN+ON7Y/+7x0uM8CANrI1NSrPz/dzXb3iBLz8/8lbjzZsiMnO0OE3KeXungGFiFkzeZffi6HHyaaJOC5Jn2O3Mpnsm7sQjfkNnyuUBI2VU/3avXdJrNx4Cz524sBXPCp+fuJfX848N2pWsuTy1ck73hje8pjK0f8dD6PWrMk4pEF4s9Pd1fVqQBArTWOJ26ru9rb2m3ckRY55jsghEZmk2njUNwQgcTLjXbRaPhQ6IY8Q8ifIORP0Bl6+g0qqwAOO4gEib99Jt/3hwMXGn6yPLnu6SfefGvHgoX3jepWoULO5lWRq+8Pr73ct3Dm2Hf5VLTrj/6ofusFLE1AyF0G9ki0CdpEQjHunehfMHQLUiJBuEgQbhXAUQP/pRhcDheILPt9sCeFIuGzz0t/89aO6fFxY75z7CTBsw+PfXWaVm8uOda+85nJuOEBQu5jGZ9RFIdk2jBo80cYugU1uwEcidsoijIzDP6dDgxqtebgR2XF+z9m+30AgEQi3vTihh27tsdMjvbi2Ggj8+GXbS+mRA7X4w0hNHY2G+pwKK5IEB4ikGABqf/C0A0NBHB6urffoDaaDOQkw8ZtuAzOnw3X72Pry5u3v/oy6ffhXUdOdS1MCJkbN/bJVoSQDQaAA8BYxm1cDk8kkIgE4Zhp83cYuqEBZA2cwajtN2hoY//dFygK3DyLqtH0aDQ9ACAUCqOjPb1KvaXlNjmIixt5oRU7VAcX/+0z+fenfmhtbVuzNvXJdY97/isinO/34UU19WrayLi8qS9CQWtwyfKQ9p18rlAkkAj5YV4bFnIpDN3QEAJeqIAXajQZ+g1qPd3Lnh+M2ygAs6uKGTSanupvvv3sSGXVsWrL8x/se2916qPOBFLjf3rmK3ermtesTd3y0qY1T6bavf5/fjz317LPDn5Uxp55t+C3Gzc9axWZvV/wwQe//9MH+957eMVDma/srDpW/WHxB2KxRzuSD9fvY3v2S5te3ODJkTjW1Ko7c6Uv57lYbw8EoQAwkGaz+tdZyJ8gEoj5XJ/4VQ25ig/1dfu/7d17VFTnvTfw3zN7huGiDIkG4yVgoiExasGkUdS0psFTA3HGniY5AROO1iWIOcYkMJ5Vc4aVk8Uc2xUhatq3koHXN32XBRq1eV+mCceVQsoxEYz1Qo1hiRgPU6MRNAFSlWEu+/zxwHY7Vxi5zIbv5w/Xnj0Pex6yjHx5Lr8Hwo1bdPX0fmd3fHdzI2q/2x+EO32q+fHHnpJe5m/ZRERvbf+1dGfv78v8pajb19Fx9dWXfs4jY/6WTTpd7OFPj0gvt5ryPdq/f8Aqhbw3zK9JjVekp+341S+l9Ma/KcueXbwi2l+Onkhf/vQfqvf+YNkS6VHDWtft008at//i7ZDrfYykK92O3X9s37zqbl0MjkcECJ3P03H4gjatZqKgwgDNGIRViuCXignR2rg7JtwTGz3FY6T9Zm4LKfpLuS1/y6bGY7UdXV9uNeVvNeV3dH35508+WLtuNRG98FzOwf+sDfakEL25bcfBmtoV6WknTn+y1ZT/4kvr91aV7f19GRG9tf3Xh+oPyxsfqj/Mc9sfqvd2dH3JG584/cmK9LSDNbVvbtshtfzmm2+J6NFFj/CXd0+dQkSn/npLDY5h8v4B648eW/mTp1bLc1vGyh/X/OnA//ugItxym8Mplv9nR9aPJiG3AYRG7B9g8/gnWCNoJ0ROvnNiQrT2DuS2sQrRDYKLUEdPj
Iq/Y8I90do4Fbv1Zy3rrygy4AzX3f2dlNu2mvJnzb5X/u7c+XO27zDzQbgXnsuRFqINodOnmvm85y+L35BPy654Mu0N82tE9M7u/yNvz1++9fY2+eDZjBnTtv5bPhG9u6fi9KlmfvPOO+8goo9r/4u/PH/uv4loYeojQ/4tSOw99qrf7X94/g9z170sr9OW+fwzh49+9NvflX7/0QXD9+khKz/Y8fj3Js6eGjnaHQFQGFHsW8zGiMn/zWXEIiMm3jFhhi5mGs6MH/MQyWGgBJU6WntHtPaOXuf1Xud1u+MaP9KeiIjEmxku2GK42o/+zC/+ZXOOvzbZazP55OnRI8eGfNFbS0srEa1IT/N+8or0tNdN2w7W1F64cJG/e671PJ8bfTLjHzwaz50/h18cP97Er+fOn5O/ZVP+5tcu/O2iThf7umlb/pZNwxSefNb70EZq1657ftTrfQR28FinLkZIfRA/XQAGgS9TYcxzgpQvUNZGTMC+0fEDo24waBHqaD4gPzHqLq8tS9L2Jr/LKA/sqyYiy55dARbvz5gxjQ+88cZD69LFy0S0ZOki77ekIUA+YEZEra1fEtGK9DSfG0X5KN1fT34u3dlqyt/7+zJedMOyZ5f3srkAzracW/N8Xu66l+V7Qr21X+4w//ubj8z/ofnf35RyW/yUu7b8/OXTLUfMvzCFc247df76qbYbqx/HaVcAAyIFMo/lxYJKE6O9886JCbHRU1DvY7zBqBuEiBHj9URiIl12xzW74zupJhwRkch/OyRiTHS7WX9luAsXLvJBrKSk2YGf/9gPUt/a/uuDNbUdHVeD1teQanYEENro3bmz58lPziOiWfffS0Tv7qnYvsMs3VzxZBo9OegPsvfY1zyfd7blHBFlrv6pz9Vp/up9bNi4du2658Ok3kcAV7od7x36ZutzOO0KIAhpK5hXYlNHaGIiNRMFFUpYj1+IbnC7VEyIioiNioh1uZ29jmt25999ZjjGmOh2f33pMr+tuyPIUcfTpvfVjLDbAw1BcU0nTv3U8ELgNtJOz4mxE4jo8KdHXnxpvb/Gp/76BW8sr2TrLTpqyArJmraaeW7TRmq9DxL9/NQXb+945/0DVvnNMKz3EcB1u3v3H9tfXBl/O0fUA4xhoigylYqvG/aZ2LTqCWoh9BPnYMxAdIMhI6jUUVpdlFbnM8OJokiM3bjeV+z39OfN0qSkT1euXOUXX1+6HHTAbNr0qXzuMnAbfjHnoQeI6GBN7V+OnvBYiOY9dCevVxKAtDYuNHV/qn93z+/4dfFbRfIZT3/1PnI3rs1Y+eOQP3HklR/syHhUN/VO/OABuIVIooqpRP5brojEBsEhusHQk2c4u+Pvvc5rt4zDERHRC8/53aPgQUp7AcyafW+AITQP3390Aa/rkb78acueXd9LnqeN1H596XLzF2fyNwfJf8Oh/XLHSy/+K7/OWPljaRTt/QPWt3e8I983yhvkbly79LHUke7l7dl36Op9d2sfuX/CaHcEICyIss1czKvAKhIbBIboBsNIUKmjtXHR2jiX29nrvC4/X2vv78vks43eG1OvXLnKq6lJo2VDaMevfvnmth3v7qmQn6ZARLyk3Lt7KqZOmzLkH+qPaauZ7zaIn3LX9reK7D329w9Y3/zl23+zXZA340fF3580LLV8h9Wxs3+/9K1zswFbE2DcE0WmUt08IfpWGkGrUUdrNTFYxwaBIbrBSBBUar4eLja6LxLNm/fQ9Bn+Mhkj0X2ufzp1OFbf33XXpO07zGvXPX/hq4vnzp6fGDth5syEe2fNnDFj2guZOUQ0eXJfzli7brX88Ct/Qp4trfrdfmkFm/kXpvcPWN/e+Y53vY8NG9fekzAjtI8YXbYO+4dHu37+T9iaAOOVKBJT9VX08Np6z4hFaGI0QmSEJtqzaiaAH4huMKLuuSeh78oRMzEq3unq6XXecLkdt7YSibGLX13iL2b4TXg3SdXXAjD8Y4ZHwJo7f87c+XM8doPy50hDfffeNzPop4fsb7YLpq19W1N1utgtrxbKd0XodLG5G3+24cWf8VIjSnTd7i6r6Xj5J1M0alQugHFFZKQS++Oa91FVgkoToYnWCJER6uhR6B0oHKIbjKgpU/pG3Zqbm5OTk7WamBgiPp3qdNl7ndelMr+fHGqkvhlMRkT00b/d9Uyljyeu3dOx6/GLX1163bQt8EfP/95DQcfGzrWe5xd3xU/mF3w76uumbT7X0vFDrngJuhC8tPFfpawmD20KqvcRgMMp/uaDy88/MWlyLGZ/YDyQxTVfpzwzpopQR2uESI06CkdUwe3A3x4YUTqdzmQymc3mioqKzMxMfpNPp/Jrh8vucN44f/4s39pp+ElG31f+w390dP3HzQcx9uX/enrR1uOPPnAvET340AN/qN4b+KMffOiBoN3jQ25vmF+TygXP6f8qn+XlDn96hIge/n5y0Cd7e+c3ezz2jRLR/UmzNr+yQSn1PgLb98k38xOjHpwxZPVTAMJPkLhG/SvYItTR2HMAQ8VvyXuAYdLU1JSSkkJEFoslJ8f3PtPCwkKz2fzUyoz39u/lYc6ryZ8LdOv+78LCIx/97D7yMR8RgtOnmvnhqidOfyKNz3V3fzfrnmQisuzZ9Y9P6+Xt/3L0RPryp4noi9ajQYsGS9aUnCMik55+9IOV3qcmfP/RBVqtloiWPLaIiLSR2qzVT4fz6Qj+1DV1/a3DsWb55NHuCMBQEolY/1aDAM00glajjlILWo06CuccwJBDdINRUFJSYjQaiai4uPjZZ59NSEiQ3mpqaiotLS0tLSWiM2fOJCUl8fsOZ4/DdUOKcbUv35f5Lv3z/i9Lbj1ZlBGJjJEY5BzV06eapRNIuUP1h3lR37fe3pa9JlP+1m9+Vc5nY2v+dECqA3eu9Tw/7fQN82sDr0vC2XvsTy5/2qPqhz/aSO3pliPKWu7Weqln/6FvCn46FUvcYAwQRVHFmBjsV0TENRgxiG4wOqT0xhUXF3d2dprNfWv29Xp9UVFRcrLviUjHmZIfPmhsXFdxZecSaW2cT8zXUaoXLlxcMPcxIlq7bjXfhSCtk/OZw7q7v3sxN5/Ppa5IT1uydNH5L/+bbzvN37JpUKeUcp+f+uLDP35ERK1nv+SbST8/9UWAYxsOH/1IQTVBrnQ7dv+xffOqu3Ux2C4HSiX2HR4a6OcjI6ZWR2oELeIajDBENxg1TU1N+/fvl+Iap9frV69enZ6ertP5PSmrZgPLsCzZefbTl2eTy+1wOHtc7l6Hq8e78K8XJpLIZANpkvwtmwyr0j2G4iTd3d/9//c/8CjY6z2Fevv46jd7j/0vR0/yOwkJ0xW09M3hFEv+cOmZH9w5e2rkaPcFYKBEIiKRMRUF+4EoqNRqIVKtiuChbSQ6B+AF0Q1Gn81m4xeRkZHx8fFBWsuD261E0e1w9ThddofL7nT2+FwyLHfhwiWe5GJjJ0r7EoJ9yUV+cTvHXo1huz9oXzArKvXBAf3HBBhNosiYioKGNSKNoFWrI9WqCI06CqXXIBwguoGytO5aev8rh3M/FN9JD9a0b0BOdDhdvb42Onjj8yNi4HVy4M/BY51Xv3OtfhynJkCY8rmCwptaiFALWoFpMLQG4QnFQUBJWneteeUwLdm5JWhuIyJBpREiblYUc7jsTpfd7Xb4n1rt2+Hf/5IRI3K7iSHJBXfq/PVTbTeMPx36U8sAQiCSqCLPvQX+cpug0qgFraDSaIRIjRpz/RDuEN1AQWq2v3KYluz8rddU6UBoBK38F2iHs8fp7nW7HU63w8+YnEgi3ZrbEOZ8u9LteO/QN1ufwyQyjA7voMaI8RVsPtvzcTUVU2uESLVaix0GoCyYMAXFqNnAMiw0oLnSwXO5HU5Xr8vd63DZXa5et+ga2NfxM27G9f9E1+3ukgOX1j9519Q7UXEURoQosgFU65DwraBqIUJgGkGIwBwoKB1G3UAhWneZLUS5Hw5HbiM+u6rSEMXwl6Lodrp6Ha4eUXQ53Q7/mx58/urDiESRxHHyq3z5wY6MR3XIbTAsRJEYY4zdMtnJAo2oEZFG0KpUGkGlVgtaQRWBU6dgjMGoGygD357g
651hGobz5BZdLpdjAGHOJ57h+tqPpX0Q+w5djdIKKxfGjXZHQPlEkRjz+J9lIPiRoComaIQolUoQVDgzF8Y4RDeAEPEw53R9uHPRAAAP9klEQVTZRXI7XHa32+lyOwb5jL6fUmL/7lZlRbpjZ//+afO1zYYpo90RUJT+gTRRpMEeYSeo1CqVRi1EMFJphEiVSo0RNRiH8JceIEQqJqjUgsd+NKerVxTdDtcNUXQ73Q6nyx7wvAdpSfXNP/ll3wRR3zhEOLJ12D882vXzf8LWBPCtf82A1ygaY+R/s+fNVsTU6kg+nCaoIlQqNdaoAXCIbgBDSS1EEJFHnnO47CSKDlcPkeh09YokBqsz1/9zzTO3yX4Kjmqqu253l9V0vPyTKTildNzrGyz2HkVjA5761AhaYiqezNSClpEKuz4BAsCEKcCocTh7iMjhukE83hENrHSwP6zvD1Ec1rlXh1PcVf31yoVxD86IGrYPgXDBB88YkTj4VWge1EIEY4JaiGDEBFWEigmIaAAhQHQDCDuekU5082B3e24GOyK6nZV1FX++OmmisOIRbE1QNo+/Af2F0Oh2whknqDQqlVqt0jCm6otoQgRjqtt8LABwmDAFCDt8vtVj1lUk0em0E5HT3SuKLuofqHO7HS63cwBP9Sin4DO3yW/63g9b19TlcIrIbWFOFEUmzaczJj9QQLry+BswqPKEjKnUfH5TpWFMRcQ0QiQRCYIGp3wCDDdENwBlYMT6Ih35PqiHr6hzi30bXV1uh1t0D3LEzscPb/kP+NZLPZ+duVZw87QrH1Gv/xqzYLfFx39B2erG/oVlUltPTL4OMqSpFT5yRnwhGhFjgloVQUSY4gQYdYhuAGNE4P13LrfD7eZjdTeI1xx2O4hIFF1+TnT1dKXbUfnx1c2r7pZtTfCXCYL/aJetnZLueEeM8I2AA+yZj2a3jIFJrTz5eLgsjd3+QhdeZYOIVEzg9TX4zCYxho2cAGEO0Q1gXOg/LsJzHlaOV6rj19K0LBG53I4eh+t/H/w6e/k0XczQZCnvavh+zqUIUwPsmY9mw7a8mG8C4Ney+NU3lUlEKFcLMDZgmwIABPfm/s8XPTB52fy7pTvS2jsikmZpOe8p2tvbOTvu8GIZnnf6qZhaSmBYWwYwDiG6AUBwlztvTIkbylIgfBftLXdcPR5Db7wG3gAeNQq5UD7EFYD35COvW3bLHaweA4DBQHQDAAAAUAwU2gEAAABQDEQ3AAAAAMVAdAMAAABQDEQ3AAAAAMVAdAMAAABQDEQ3AAAAAMVAdAMAAABQDEQ3AAAAAMVAdAMAAABQDEQ3AAAAAMVAdAMAAABQDEQ3AAAAAMVAdAMAAABQDEQ3AAAAAMVAdAOAwbHZbHV1dV1dXaPdEQCA8QjRDSAc2Ww2m80WnvFo3759aWlp5eXlo90RAIDxCNENIIzYbLaSkhLGWGJiYmJiYlxc3MaNG+vq6ka7XwAAEC4Q3QDCRUtLS2JiotFoJCK9Xm8ymYiotLQ0LS1t5NNbXV0dIiMAQBhSj3YHAKAPD20mkyknJychIYGIioqKmpqaSktLr127NpI9sdlsaWlpRNTW1sZ7AgAAYQLRDSAsNDU1Wa1WIsrOzpanpeTk5N27d4fnojcAABh5mDAFCAtXr17lF0lJSd7v6nS6AF/L9zQMS7cGZtQ7AAAwfiC6AYSXlpaWAbZsamoqLCyU9jQwxjZu3NjU1OTdsqysjDHmc+2awWBgjDU2NvKXjY2N/IH8JX8sY8xgMPjsQ11dncFg4B0wGAxYHgcAMNwQ3QDCwrx58/jFjh07BjI9WlVVlZKSYjabichkMkl7GlJSUqqqqjwad3d3+3sOn6W9fv164I/jzeQ6OzsLCwvT0tKkt6xWq/wlAAAMB0Q3gLAQHx9vsViIqLS0NDs7O/DwVV1dXVZWFhFVVlaKolhUVFRUVCSKYmVlJRFlZWWFPPqVmpoqimJbWxt/2dbWJvbzaGk2m81mc3V1NX+3s7OTx0eDwYCVeQAAwwfRDSBc5OTkFBcXU//wVYD5x507dxKRxWLJzMyU38/MzOT5iTcYbidPntTr9fxap9O99NJL/Lq5uXkEPh0AYHxCdAMIIwUFBSdPnszLyyNZgJMWonHSXtRVq1Z5P+GZZ57hXzvc+wZMJlNycrL8Tnx8PE9yQadfAQAgZIhuAOGFVwORB7jFixfLF5BJe1Hj4+O9v3zq1Kn84uLFi8Paz7i4OO+by5YtG9YPBQAARDeAcMQDXENDA39pMBikUbQTJ04QEQ923qQ8h6EvAIAxCdENIHylpqZK6c1n1Q8AABhvEN0Awlpqaiq/kOq9TZ8+nYhKS0t9tm9vb+cXkyZNGv7eAQDASEN0A1CG2NhYfjFz5kx+IaU0uUuXLvELadGbhM+0AgCAoiG6AYQFf/Oh0v358+fzi/vuu49f+Cwdsn//fiLKy8uTb2Lgsa++vt6jsXfxXg89PT1Bew4AACMJ0Q0gLKSkpPBCbvJ6tvyoKyLS6/XSzGl8fDwv/5aVleVRN6SsrIyfr/Dqq6/K7y9cuJCIrFarPKuVlZXxur7eEhIS+IWU9lBlFwAgTKhHuwMAQERUXFxsNBp9niKl1+vLy8vldwoKCjo7O81m8+LFi/V6PS/JYTQa+bu1tbUeZ9gnJyebTCaz2ZyVlVVRUbFs2TLe2GKxdHd3S18oZ7FYcnNzc3Nzjx8/PnnyZLPZ7H2gAgAAjDxEN4CwUFBQsGDBgn379nnsP6isrExPT9fpdB7ti4qK5s6dW1FRYbVapcBnMpmys7M9cpvUPi4ujqdDq9Wq1+tfeeWVJ554oqyszGd/cnJyeKrj/ZFOTZBIa+8AAGAkMfwmDRBWurq6pNlJaeIyAKnem06n8054/h4+kCdL7SMjI32W/wUAgJGH6AYAAACgGNimAAAAAKAYiG4AAAAAioHoBgAAAKAYiG4AAAAAioHoBgAAAKAYiG4AAAAAioHoBgAAAKAYiG4AAAAAioHoBgAAAKAYiG4AAAAAioHoBgAAAKAYiG4AAAAAioHoBgAAAKAYiG4AAAAAiqEe7Q4AgFK1t7f39PQQUUJCQtDGXV1dXV1dgRtXVVXV19d/9dVXer1+1apV8fHxQ9hbAICxAaNuAHBTU1OTwWBgjJWVlQVoVlVVZTAYpkyZkpiYmJiYyBgrLCxsaWnx2bixsXHjxo1xcXFS45KSkvb2do9mhYWFWVlZDz/88OrVq3Nzc9evX8+jHgAAyDFRFEe7DwAw+rq6usrLy41GI39ZXFxcUFDgs1l2drbVaiUik8kUFxfX2dlpNpv5uydPnkxOTpa3r6qqysrKkp5ZX1/Pv1av15eXl0vjak1NTSkpKZWVlZmZmUTU2Ni4ePHi2traJ554Yli+WwAAxcKoGwBQY2Njdna20WjMy8sL3PK9996zWq16vb6tra2oqKigoKCoqKizs5N/YWFhobxxXV0dz221tbWiKBYUFFRXV7e1ten1eqvV+vrrr0str169SkRLlizhL6dNm0ZEJ06cGNLvEgBgLEB0Axjv+BCX1Wqtra3dvXt34Ma5ublElJOTI1+yptPpXn31VSKyWq02m026v3PnTiKyWCzywbOEhISioiIiKi0tbWpq4jcnTZpERAcPHuQvW1tbiWjp0qVD8O0BAIwt2KYAEI74DoCBLP8fEsXFxevXr9fpdEF7xS8eeOABj7ciIyM97rS0tPC50VWrVnm8JU2qfvbZZ/w6OTnZZDLl5ubabLa4uDij0WgymVJTU0P6bgAAxjKMugGEl7q6OmkHAF/Rz1frt7e3M8YYY0O+eD81NbWgoCBobiMiaWna8ePHPd769ttv+YX0nDNnzhCRXq/3uVG0uLjY4zlFRUXV1dVxcXFEVFlZyUfmAADAA0bdAMJIYWEhX/Kv1+uXLVtmNBqNRmN9fX11dTUvw0GybOSPVIYjgJDH8yorK7OysrKysubMmSMNnrW3t/NVbtXV1VL3+IbTZcuW+XxOUlISEZWWlsqnaPV6fWi9AgAYPxDdAMJFXV0dz20NDQ18rrCgoKCsrCw3N7eqqoqPXZlMpqDPOXbsWFpaWuA2IW/ezMzMrKi
osFqtKSkp1dXVer3eZrNt2rTJarWaTCZ59urs7AzwnJiYmBA+HQAAEN0AwoW0qF++xisnJ8dqtVZUVPBUNHfu3KDPmTFjBp+ODNwm5H5WV1eXlJQYjUaDwSC/6TFmJlUMCcxms43Ykj4AgDEA0Q0gLNhsNr6o33uGMScnx2AwTJ8+nYjmzJkT9FFJSUk+S7INofXr18vLuen1eoyiAQCMDGxTAAgLvBwG9S8Ck+PbOUtLS4lo5syZI9svH+rq6uLi4sxmc3FxcUNDQ15entVqTUtLKyws9D4jAQAAhhaiG0C4k+pu5OXlDWQf6LCqqqriC+kaGhoKCgpSU1N3797d0NBARGazWX56VdDqvhxmSwEABgUTpgCKkZGRMZBmUkG1AJ599tkQMpPNZuOnI3gceJWamtrW1paYmGi1WmtqavhhVrNnzx7s8wEAIChEN4BwJ5VM8zge1J8LFy5IR5H6s2DBghCiGz/tIC8vz7snCQkJFoslNze3oqKCR7fY2FgiMhqNPhfe8UOuBrJhFgAA5BDdAMICPwmKiNrb2z1q2DY3Nw/qUfPmzautrQ3aZlDP5Lq7u8n/cNqsWbOISBrwmz9/Pr/w/o6IqL6+nogWLlwYQjcAAMYzRDeAsCDtPzhy5IhHlY2Kigp+8e233w5kqCw+Pj60mm0DVF9f73Mg7eOPPybZQJq0Gbauro6Pw0kaGxt5wlu0aNHw9RMAYEzCNgWAsKDT6XgxNoPBID/BvbCw0Gq18iX/n3322aj1j4iIli9fTkRWq7WqqsrjLavVymuFPPXUU/yO9B1lZWU1NjZKLVtaWrZt20ZExcXFPs/IAgCAAJgoiqPdBwAgImpvb1+/fj0fjjKZTHFxcfX19Vartbi4ePny5SkpKfz+lStX1qxZk5qaStS6a+n9rxzmX537ofhOekifa7PZEhMTAzSQTncg2VFd1H8OKRHxfhKRxWLJycmRvrCrqys7O5u/xY/2am1t5VVOTCYTTikFAAiFCABh4/Llyx4HIVRXV/O3KisrpZsnT54URfHDXKIlO8/yt8/uXCJdD/5DA/8r0dDQIG9fW1vrXfgjLy+vtrbW++GdnZ0Wi8WjcWVlZUg9BQAAEaNuAGFHOj/eY2Vbe3t7T0+PTqfT6XRENRtYhiX0sbYh6yQR9XcpEGkWGIXcAABuB6IbgFLVbGAZltuZKQUAAOXBNgUApUp/Rzy7cwlZMhhjjG2oGe3+AADACMCoG4Dy9W1XwPgbAMDYh1E3AOWb/fJvdy4h+ryldbR7AgAAwwzRDUCZWnctvTlJ2vrBe4eJ5iXh1FAAgLEOE6YASiWv6obdCgAA4wSiGwAAAIBiYMIUAAAAQDEQ3QAAAAAUA9ENAAAAQDEQ3QAAAAAUA9ENAAAAQDEQ3QAAAAAUA9ENAAAAQDEQ3QAAAAAUA9ENAAAAQDEQ3QAAAAAUA9ENAAAAQDH+B+vmdXPyxVyAAAAAAElFTkSuQmCC\" 
border=\"0\" hspace=\"0\"></P>
<P>With these calculated variables, the zenith angle (&#947;) and azimuth angle 
(&#945;<SUB><FONT size=\"2\">s</FONT></SUB>)             of the solar radiation are 
calculated depending on the current time with the following equations (VDI 
6020):</P>
<P><BR></P>
<UL>
  <LI value=\"1\">Zenith angle: 
  cos(&#1012;)=sin(&#948;)*sin(latitude)+cos(&#948;)*cos(latitude)*cos(&#982;)</LI></UL>
<P><BR></P>
<UL>
  <LI value=\"1\">Solar Altitude: &#947;=90°-&#1012;</LI></UL>
<P><BR></P>
<UL>
  <LI value=\"1\">Azimuth  Angle:  
  &#945;s=180°+sign*acos((sin(&#947;)*sin(latitude)-sin(&#948;))/(cos(&#947;)*cos(latitude))), with 
  sign=-1, if RealLocalTime&lt;=12 h and: sigh=1, if  RealLocalTime&gt;12 h 
</LI></UL></BODY></HTML>
"), experiment(
     StopTime=1,
     StartTime=0,
     Tolerance=1e-06,
     Interval=0.002,
     __esi_Solver(
      bSplitCodeGen=false,
      typename="CVODE"),
     __esi_MinInterval="9.999999999999999e-10",
     __esi_MaxInterval="0.002",
     __esi_AbsTolerance="1e-6"));
  end Environment;

  expandable connector Bus "Data bus that stores weather data"
    extends Modelica.Icons.SignalBus;

    Modelica.Units.SI.Temperature TDryBul "Dry bulb temperature";
    Modelica.Units.SI.Temperature TWetBul "Wet bulb temperature";
    Modelica.Units.SI.Temperature TDewPoi "Dew point temperature";
    Modelica.Units.SI.Temperature TBlaSky "Black-body sky temperature";

    Real relHum(final unit="1") "Relative humidity";

    Real HDirNor(final unit="W/m2") "Direct normal solar irradiation";
    Real HGloHor(final unit="W/m2") "Global horizontal solar irradiation";
    Real HDifHor(final unit="W/m2") "Diffuse horizontal solar irradiation";

    Real HHorIR(final unit="W/m2") "Horizontal infrared irradiation";

    Modelica.Units.SI.Angle winDir "Wind direction";
    Modelica.Units.SI.Velocity winSpe "Wind speed";

    Modelica.Units.SI.Height ceiHei "Cloud cover ceiling height";
    Real nOpa(final unit="1") "Opaque sky cover";
    Real nTot(final unit="1") "Total sky cover";

    Modelica.Units.SI.Angle lat "Latitude of the location";
    Modelica.Units.SI.Angle lon "Longitude of the location";
    Modelica.Units.SI.Height alt "Location altitude above sea level";

    Modelica.Units.SI.AbsolutePressure pAtm "Atmospheric pressure";

    Modelica.Units.SI.Angle solAlt "Solar altitude angle";
    Modelica.Units.SI.Angle solDec "Solar declination angle";
    Modelica.Units.SI.Angle solHouAng "Solar hour angle";
    Modelica.Units.SI.Angle solZen "Solar zenith angle";

    Modelica.Units.SI.Time solTim "Solar time";
    Modelica.Units.SI.Time cloTim "Model time";

    annotation (
      defaultComponentName="weaBus",
      Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
              100}}), graphics={Rectangle(
            extent={{-20,2},{22,-2}},
            lineColor={255,204,51},
            lineThickness=0.5)}),
      Documentation(info="<html>
<p>
This component is an expandable connector that is used to implement a bus that contains the weather data.
</p>
</html>",   revisions="<html>
<ul>
<li>
September 22, 2023, by Michael Wetter:<br/>
Declared the variables that are on the bus.<br/>
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1798\">IBPSA, #1798</a>.
</li>
<li>
June 25, 2010, by Wangda Zuo:<br/>
First implementation.
</li>
</ul>
</html>"));
  end Bus;
 annotation(dateModified="2023-05-03 12:45:44Z");
end Environment;
