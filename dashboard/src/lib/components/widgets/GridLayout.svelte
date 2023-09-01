<script>
    export let columns;
    export let cellHeight = undefined;
    export let cellWidth = undefined;
    export let showLines = false;
    let gridWidth;
    let gridHeight;
    let unitHeight = undefined;
    let unitWidth = undefined;
    let columnsCSS;
    let rowsCSS;
    $: {gridWidth, gridHeight
        if (cellWidth == undefined || cellHeight == undefined) {
            unitHeight = gridWidth/columns;
        unitWidth = gridWidth/columns;
        columnsCSS = `repeat(${columns}, ${unitWidth}px)`
        } else {
            unitHeight = cellHeight;
            unitWidth = cellWidth;
            columnsCSS = `repeat(100, ${unitWidth}px)`
        }
        rowsCSS = `repeat(100, ${unitHeight}px)`
    }
</script>
<div bind:clientHeight={gridHeight} bind:clientWidth={gridWidth}
    style="
    display:grid;
    position:relative;
    width: 100%;
    height: 100%;
    grid-template-columns:{columnsCSS};
    grid-template-rows:{rowsCSS};

    {showLines ? `
    background-size: ${unitHeight}px ${unitWidth}px;
    background-image:
    linear-gradient(to right, grey 1px, transparent 1px),
    linear-gradient(to bottom, grey 1px, transparent 1px);`:""}">
<slot/>
</div>