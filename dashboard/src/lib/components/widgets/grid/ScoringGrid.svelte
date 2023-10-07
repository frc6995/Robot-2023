<script>
    import Node from "./Node.svelte";
    export let selection;
    export let selectCallback;
    export let cubesScored;
    export let conesScored;
    export let showLinks=true;
    let leftOfLinks = [];
    $: {
        leftOfLinks = [];
        let scored = cubesScored.concat(conesScored);
        for (let row = 0; row < 3 ; row++) {
            for (let column = 0; column <= 6; column++) {
                if (scored.includes(row * 9 + column) &&
                    scored.includes(row * 9 + column + 1) &&
                    scored.includes(row * 9 + column + 2)) {
                    leftOfLinks.push(row * 9 + column);
                    column += 2;
                }
            }
        }
    }

</script>

<svelte:options tag="frc-node-selector"></svelte:options>

<div style="display:grid;
    grid-template-columns: repeat(9, calc(100% / 9));
    grid-template-rows: repeat(3, calc(100% / 3));
    height:100%;
    width:100%;
    flex-direction:row-reverse;
    grid-auto-flow: dense;">
    {#each {length:27} as _, i}
    <Node id={i} selection={selection} on:click={()=>selectCallback(i)} cubes={cubesScored.filter(x => x==i).length} cones={conesScored.filter(x => x==i).length}></Node>
    {/each}
    {#if showLinks}
    {#each leftOfLinks as id}
    <div style="
        grid-column: {id % 9 + 1} / {id % 9 + 4};
        grid-row: {Math.floor(id/9) + 1};
        border: 5px solid black;
        border-radius: 10px;
        margin:5px;
        box-sizing: border-box;
        pointer-events:none;
        ">
    </div>
    {/each}
    {/if}
</div>
