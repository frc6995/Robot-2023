<script>
    export let id;
    export let selection;
    export let cubes=0;
    export let cones=0
    let background;
    $:  {if (selection == id) { background = "green"; }
        else if (id >= 0 && id < 9) { background = "lightgray";}
        else if (id % 3 == 1) { background = "purple"; }
        else { background = "yellow" }    
    }
    function mapNumber(id) {
        let column = 8-(id % 9);
        let row = Math.floor(id/9);
        return row * 9 + column + 1;
    }
    $: num = mapNumber(id)
</script>

<div key={id} 
    style = "background-color :{background};
            color:black;
            font-size:2em;
            height :100%;
            flex-grow :1 ;
            border:{cones + cubes > 1 ? '1px solid black' : '1px solid black'};
            grid-column: {9- (id % 9)};
            grid-row: {Math.floor(id/9) + 1};
            box-sizing: border-box"
    on:click
    on:keypress={()=>{}}
  >
  <div style="text-align: center; margin-block:auto;">  {num}</div>

  <!-- empty on:click binding above passes click events to be bound on the Node component -->
  <!-- <svg viewBox="-2 -2 28 28" style="margin-right: 8px; width:100%; height:100%; pointer-events:none">
    {#if cubes > 1}
    <polygon points="8,8 8,20 20,20, 20,8" style="fill:rgb(150,0,255);stroke-width:1;stroke:rgb(0,0,0)"></polygon>
    <polygon points="4,4 4,16 16,16, 16,4" style="fill:rgb(150,0,255);stroke-width:1;stroke:rgb(0,0,0)"></polygon>
    {/if}
    {#if cubes == 1}
    <polygon points="6,6 6,18 18,18, 18,6" style="fill:rgb(150,0,255);stroke-width:1;stroke:rgb(0,0,0)"></polygon>
    {/if}
    {#if cones > 1}
        <polygon points="13,8 11,8 8,22 5,22 19,22 16,22" style="fill:rgb(255,200,0);stroke-width:1;stroke:rgb(0,0,0)"></polygon>
        <polygon points="13,4 11,4 8,18 5,18 19,18 16,18" style="fill:rgb(255,200,0);stroke-width:1;stroke:rgb(0,0,0)"></polygon>
    {/if}
    {#if cones == 1}
        <polygon points="13,6 11,6 8,20 5,20 19,20 16,20" style="fill:rgb(255,200,0);stroke-width:1;stroke:rgb(0,0,0)"></polygon>
    {/if}

</svg> -->
</div>