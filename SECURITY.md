# Security

frontier-fly is a static site: everything runs in the visitor's browser, and there is no server, account, or stored data. The parts that could still matter are the pages themselves (for example, a crafted share link that runs script) and the CI workflows that score benchmark entries from pull requests.

## Reporting a problem

Please report it privately through GitHub: on the repository's **Security** tab, choose **Report a vulnerability**. If that button is not available, open an issue that says you have a security report, without the details, and the maintainer will arrange a private channel.

You can expect a first reply within a week. Fixes land on `main`, which is what the live site serves.
