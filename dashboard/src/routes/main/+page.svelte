<script lang="ts">
    import Alerts from "$lib/components/widgets/Alerts.widget.svelte";
	import Camera from "$lib/components/widgets/Camera.widget.svelte";
	import GridItem from "$lib/components/widgets/GridItem.svelte";
	import GridLayout from "$lib/components/widgets/GridLayout.svelte";
    import BooleanBox from "$lib/components/widgets/BooleanBox.widget.svelte";
    import "@frc-web-components/fwc/components/field/field"
    import "@frc-web-components/fwc/components/field/field-robot"
	import { readable } from "svelte/store";
	import NT from "../../util/NT";
    let cameraIP = "";
    $: ntConnected = readable(false, function start(set) {
        const interval = setInterval(() => {
            set(NT.nt.isRobotConnected());
        }, 1000);

        return function stop() {
            clearInterval(interval);
        };
    });
    let controller1 = NT.NTBoolean(true, "/DriverDisplay/controller1");
    let controller2 = NT.NTBoolean(true, "/DriverDisplay/controller2");

    let robotPose = NT.NTDoubleArray([0,0,0], "/DriverDisplay/field/Robot")
    let targetPose = NT.NTDoubleArray([0,0,0], "/DriverDisplay/field/target")
    let isBlue = NT.NTBoolean(false, "/DriverDisplay/alliance")
    let fixFieldHeading = (num, i) =>{
                if (i == 2) {return num / (180/Math.PI)}
                return num;
            };
    let setIsRealRobot = (isReal)=>{
        NT.setIP(isReal ? "10.69.95.2": "localhost");
        cameraIP = `http://${isReal ? "10.69.95.11:1185" : "localhost:1181"}/stream.mjpg`
    }
    setIsRealRobot(true);
</script>
<main style="width:100vw; height:calc(100vh - 200px);">
<div
    style="
    display:grid;
    position:relative;
    width: 100%;
    height: 100%;
    grid-template-columns:repeat(9, calc(calc(100vh - 200px) / 4));
    grid-template-rows:repeat(4, calc(calc(100vh - 200px) / 4));
">
<GridItem x={0} y={0} width={6} height={4}>
    <Camera name="Driver Cam" data={cameraIP}></Camera>
</GridItem>
<!-- <GridItem x={7} y={4} width={1} height={1}>
    Is Real Robot
    <input type="checkbox" value={NT.ip} on:change={(e)=>{setIsRealRobot(e.currentTarget.checked)}}/>
</GridItem> -->
<GridItem x={8} y={0} width={2} height={4}>
    <frc-field rotation={$isBlue ? 90 : 270} rotation-unit="deg" style="width:100%; height:100%;">
        <frc-field-robot
        pose={($targetPose).map(fixFieldHeading)} 
        width={0.7874} length={0.7874}
        color={"red"}></frc-field-robot>
        <frc-field-robot
        pose={($robotPose).map(fixFieldHeading)} 
        width={0.7874} length={0.7874}
        color={"green"}></frc-field-robot>
        

    </frc-field>
</GridItem>
<GridItem x={7} y={1} width={1} height={1}>
    <BooleanBox value={$ntConnected} label="Connection"></BooleanBox>
</GridItem>
<GridItem x={7} y={2} width={1} height={1}>
    <BooleanBox value={$controller1} label="Xbox"></BooleanBox>
</GridItem>
<GridItem x={7} y={3} width={1} height={1}>
    <BooleanBox value={$controller2} label="Keypad"></BooleanBox>
</GridItem>

</div></main>
<input type="text" value={cameraIP} on:change={(e)=>cameraIP = e.currentTarget.value}/>