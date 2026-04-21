package src;

import org.car2x.veins.nodes.RSU;
import org.car2x.veins.nodes.Scenario;

network SimpleIntersectionScenario extends Scenario
{
    submodules:
        rsu[1]: RSU {
            @display("p=300,200;i=veins/sign/yellowdiamond;is=vs");
        }
}
