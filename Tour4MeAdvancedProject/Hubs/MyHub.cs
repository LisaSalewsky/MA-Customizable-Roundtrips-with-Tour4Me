
using Microsoft.AspNet.SignalR;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;

namespace Tour4MeAdvancedProject.Hubs
{
    public class MyHub : Hub
    {
        public async Task SendVisitedNodesToFrontend ( List<int> visitedNodes )
        {
            Debug.WriteLine( "test" );
            await Clients.All.SendAsync( "ReceiveVisitedNodes", visitedNodes );
            Debug.WriteLine( "end" );
        }
    }
}