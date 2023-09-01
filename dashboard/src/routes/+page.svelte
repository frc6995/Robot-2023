<script lang="ts">
    import TimerWidget from "$lib/components/widgets/Timer.widget.svelte"
    import ScoringGrid from "$lib/components/widgets/grid/ScoringGrid.svelte"
    import GridLayout from "$lib/components/widgets/GridLayout.svelte"
    import GridItem from "$lib/components/widgets/GridItem.svelte"
    import NT from "../util/NT"
    import Chooser from "$lib/components/widgets/Chooser.widget.svelte";
    import BooleanBox from "$lib/components/widgets/BooleanBox.widget.svelte";
    import { readable } from "svelte/store";
    NT.setIP("10.69.95.2")
    NT.setIP("localhost")
    let name = NT.NTString("", "/DriverDisplay/label")
    let max = NT.NTInt(150, "/DriverDisplay/maxTimer")
    let time = NT.NTDouble(-1, "/DriverDisplay/matchTime");
    let selection = NT.NTInt(-1, "/DriverDisplay/selection")

    let data = "/SmartDashboard/SendableChooser[0]"
    let autoOptions = NT.NTStringArray([], `${data.replace(/\/$/, '')}/options`);
	let autoSelected = NT.NTString("", `${data.replace(/\/$/, '')}/selected`)
	let autoActive = NT.NTString("", `${data.replace(/\/$/, '')}/active`)
    $: ntConnected = readable(false, function start(set) {
        const interval = setInterval(() => {
            set(NT.nt.isRobotConnected());
        }, 1000);

        return function stop() {
            clearInterval(interval);
        };
    });
</script>
<main style="width:100vw; aspect-ratio: 16 / 9; overflow:hidden; box-sizing:border-box">
<GridLayout columns={9}>
<GridItem x={1} y={1} width={9} height={3}>
    <ScoringGrid selection={$selection} selectCallback={(i)=>{}} conesScored={[]} cubesScored={[]}></ScoringGrid>
</GridItem>
<GridItem x={6} y={4} width={4} height={2}>
    <TimerWidget value={$time} name={""} max={$max}></TimerWidget>
</GridItem>
<GridItem x={1} y={4} width={4} height={2}>
    <Chooser options={$autoOptions} active={$autoActive} selectedStore={autoSelected}></Chooser>
</GridItem>
<GridItem x={5} y={4} width={1} height={1}>
    <BooleanBox value={$ntConnected} label="Connection"></BooleanBox>
</GridItem>

</GridLayout>
</main>
