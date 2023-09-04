<script lang="ts">
    import Alerts from "$lib/components/widgets/Alerts.widget.svelte";
	import Camera from "$lib/components/widgets/Camera.widget.svelte";
	import GridItem from "$lib/components/widgets/GridItem.svelte";
	import GridLayout from "$lib/components/widgets/GridLayout.svelte";
    import BooleanBox from "$lib/components/widgets/BooleanBox.widget.svelte";
	import { readable } from "svelte/store";
	import NT from "../../util/NT";
    NT.setIP("localhost");
    let cameraIP = `http://${NT.ip}:1181/stream.mjpg`;
    let errors = NT.NTStringArray([], "/DriverDisplay/errors");
    let warnings = NT.NTStringArray([], "/DriverDisplay/warnings/");
    let infos = NT.NTStringArray([], "/DriverDisplay/infos/");
    $: ntConnected = readable(false, function start(set) {
        const interval = setInterval(() => {
            set(NT.nt.isRobotConnected());
        }, 1000);

        return function stop() {
            clearInterval(interval);
        };
    });
    $: console.log($errors);
</script>
<main style="width:100vw; aspect-ratio: 2 / 1">
<GridLayout columns={9}>
<GridItem x={0} y={0} width={5} height={3}>
    <Camera name="Driver Cam" data={cameraIP}></Camera>
</GridItem>
<GridItem x={0} y={4} width={3} height={1}>
    <input type="text" value={cameraIP} on:change={(e)=>cameraIP = e.currentTarget.value}/>
</GridItem>
<GridItem x={6} y={0} width={4} height={4}>
<Alerts errors={$errors} warnings={$warnings} infos={$infos}></Alerts>
</GridItem>
<GridItem x={5} y={4} width={1} height={1}>
    <BooleanBox value={$ntConnected} label="Connection"></BooleanBox>
</GridItem>

</GridLayout></main>