using Microsoft.AspNet.SignalR;
using Microsoft.Owin;
using Microsoft.Owin.Cors;
using Owin;

[assembly: OwinStartup( typeof( Tour4MeAdvancedProject.Startup ) )]

namespace Tour4MeAdvancedProject
{
    public class Startup
    {
        public void Configuration ( IAppBuilder app )
        {
            _ = app.UseCors( CorsOptions.AllowAll );

            // Configure SignalR
            HubConfiguration hubConfiguration = new HubConfiguration
            {
                EnableDetailedErrors = true // Enable detailed errors for debugging
            };

            // Register your SignalR hub
            _ = app.MapSignalR( "/signalr", hubConfiguration );
        }
    }
}
