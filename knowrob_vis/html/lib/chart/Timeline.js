function Timeline (options) {
  options = options || {};


  this.update = function(data) {

      // TODO: Read data from the 'data' variable
    
      var container = document.getElementById('chart');
      
      var chart = new google.visualization.Timeline(container);
      
      var dataTable = new google.visualization.DataTable();
      
      dataTable.addColumn({ type: 'string', id: 'Event' });
      dataTable.addColumn({ type: 'number', id: 'Start' });
      dataTable.addColumn({ type: 'number', id: 'End' });
      
      dataTable.addRows([
      [ 'ContactKitchenTableCup', 11414, 14437],
      [ 'ContactKitchenTableCup', 21590, 35487],
      [ 'ContactKitchenTableHand', 12662, 14379],
      [ 'ContactKitchenTablePancakeMaker', 11414, 35487],
      [ 'ContactKitchenTableSpatula', 11414, 25840],
      [ 'ContactKitchenTableSpatula', 33192, 35487],
      [ 'ContactPancakeMakerLiquidTangibleThing', 17817, 29449],
      [ 'ContactPancakeMakerLiquidTangibleThing', 31467, 35487],
      [ 'ContactPancakeMakerSpatula', 27629, 29444],
      [ 'ContactSpatulaLiquidTangibleThing', 28285, 31378],
      [ 'GraspCup', 13887, 21734],
      [ 'GraspSpatula', 25427, 33833],
      [ 'LiquidTransfer', 17566, 18930],
      [ 'Main', 11413, 35487],
      ]);
      
      chart.draw(dataTable);

  }

}
