import { MailRequest } from './mail-request';

export const REQUESTS: MailRequest[] = [
    { id: 1004, timestamp: new Date(1636601300), source: 'UC', destination: 'MC', sender: 'Alice', recipient: 'Bob'},
    { id: 1003, timestamp: new Date(1636601200), source: 'ME', destination: 'MC', sender: 'Alice', recipient: 'Charlie'},
    { id: 1002, timestamp: new Date(1636601100), source: 'MC', destination: 'UC', sender: 'Charlie', recipient: 'Bob'},
    { id: 1001, timestamp: new Date(1636601000), source: 'CB', destination: 'MC', sender: 'Bob', recipient: 'Alice'},
    { id: 1000, timestamp: new Date(1636600900), source: 'UC', destination: 'CB', sender: 'Bob', recipient: 'Charlie'}
];