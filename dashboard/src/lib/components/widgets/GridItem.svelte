<script lang="ts">
    import { onMount } from "svelte";

    export let x :number | undefined = undefined;
    export let y : number | undefined = undefined;
    export let height :number = 1;
    export let width : number = 1;
    export let onAutoPlace = (x, y)=>{};
    $: autoPlace = (x === undefined) || (y === undefined)
    let css = ""
    $: {
        css = ""
        if (!autoPlace) {
            css += `grid-column-start:${x}; grid-row-start:${y};`
        }
            css += `grid-column-end: span ${width}; grid-row-end: span ${height}`

    }

    let item : HTMLDivElement;
    let relativeXPx = 0;
    let relativeYPx = 0;
    let clientHeight;
    let clientWidth;
    onMount(()=>{
        relativeXPx = item.offsetLeft;
        relativeYPx = item.offsetTop;
        let boxSize = clientHeight / height; // px / squares
        let actualX = relativeXPx / boxSize; // px / (px / squares) = squares
        actualX = Math.floor (actualX + 0.5) + 1;
        let actualY = relativeYPx / boxSize; // px / (px / squares) = squares
        actualY = Math.floor (actualY + 0.5) + 1;
        onAutoPlace(actualX, actualY) 
    }
    )

</script>
<!-- <style>
    .grid-item {
        border:2px solid red
    }
</style> -->
<div class="grid-item" style={css} bind:this= {item} bind:clientHeight bind:clientWidth>
<slot/>
</div>