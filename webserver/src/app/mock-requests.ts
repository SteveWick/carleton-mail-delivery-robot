import { MailRequest } from './mail-request';

export const REQUESTS: MailRequest[] = [
    { id: 1004, robot: 7, timestamp: new Date(1636601300), source: 'UC', destination: 'MC', sender: 'Alice', recipient: 'Bob', state: 'completed'},
    { id: 1003, robot: 7, timestamp: new Date(1636601200), source: 'ME', destination: 'MC', sender: 'Alice', recipient: 'Charlie', state: 'completed'},
    { id: 1002, robot: 7, timestamp: new Date(1636601100), source: 'MC', destination: 'UC', sender: 'Charlie', recipient: 'Bob', state: 'completed'},
    { id: 1001, robot: 7, timestamp: new Date(1636601000), source: 'CB', destination: 'MC', sender: 'Bob', recipient: 'Alice', state: 'completed'},
    { id: 1000, robot: 7, timestamp: new Date(1636600900), source: 'UC', destination: 'CB', sender: 'Bob', recipient: 'Charlie', state: 'completed'}
];